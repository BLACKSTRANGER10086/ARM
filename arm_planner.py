"""本地 3-DOF 机械臂规划与安全校验模块。

负责把 LLM 输出的高层语义 `commands` JSON 或旧式规范中文任务编译为可执行
`move_joints/gripper/wait` 任务 JSON。主要对外接口是
`build_task_from_command_plan()`、`build_task()`、`validate_task()`、
`forward_kinematics()` 和 `forward_kinematics_points()`。本模块被
`llm_planner.py`、`task_demo.py`、`web_ui.py` 调用，是项目中唯一的关节轨迹
生成与安全检查层。
"""

import argparse
import json
import math
import re
import sys
import uuid
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any


BASE_HEIGHT_MM = 260
UPPER_ARM_MM = 280
FOREARM_MM = 240
ARM_REACH_MM = UPPER_ARM_MM + FOREARM_MM
HOME_JOINTS = {"j1": 0.0, "j2": 90.0, "j3": 0.0}
SAFE_YAW_JOINTS = {"j2": 55.0, "j3": -90.0}
MIN_TCP_Z_MM = 0
J1_LIMIT_DEG = (-180.0, 180.0)
J2_LIMIT_DEG = (-45.0, 120.0)
J3_LIMIT_DEG = (-150.0, 150.0)
GRASP_HEIGHT_MM = 120
PRE_GRASP_HEIGHT_MM = 220
TABLE_HEIGHT_MM = 260
APPROACH_CLEARANCE_MM = 100
MOVE_DURATION_MS = {"slow": 1200, "medium": 900, "fast": 650}
GRIPPER_DURATION_MS = 420
CHINA_TZ = timezone(timedelta(hours=8))


class PlanningError(ValueError):
    pass


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass(frozen=True)
class ArmPoints:
    base: Point3D
    shoulder: Point3D
    elbow: Point3D
    tool: Point3D


def configure_stdio() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")


def normalize_yaw(angle: float) -> float:
    while angle <= -180:
        angle += 360
    while angle > 180:
        angle -= 360
    return round(angle, 3)


def shortest_yaw_delta(start: float, end: float) -> float:
    return normalize_yaw(end - start)


def planar_forward(j2: float, j3: float) -> tuple[float, float, float, float]:
    shoulder = math.radians(j2)
    elbow = math.radians(j2 + j3)
    elbow_r = UPPER_ARM_MM * math.cos(shoulder)
    elbow_z = BASE_HEIGHT_MM + UPPER_ARM_MM * math.sin(shoulder)
    tool_r = elbow_r + FOREARM_MM * math.cos(elbow)
    tool_z = elbow_z + FOREARM_MM * math.sin(elbow)
    return elbow_r, elbow_z, tool_r, tool_z


def forward_kinematics_points(j1: float, j2: float, j3: float) -> ArmPoints:
    validate_joint_limits({"j1": j1, "j2": j2, "j3": j3})
    yaw = math.radians(j1)
    elbow_r, elbow_z, tool_r, tool_z = planar_forward(j2, j3)
    if tool_z < MIN_TCP_Z_MM:
        raise PlanningError(f"TCP 低于基准平面: z={tool_z:.1f}mm")
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return ArmPoints(
        base=Point3D(0.0, 0.0, 0.0),
        shoulder=Point3D(0.0, 0.0, float(BASE_HEIGHT_MM)),
        elbow=Point3D(round(elbow_r * cos_yaw, 3), round(elbow_r * sin_yaw, 3), round(elbow_z, 3)),
        tool=Point3D(round(tool_r * cos_yaw, 3), round(tool_r * sin_yaw, 3), round(tool_z, 3)),
    )


def forward_kinematics(j1: float, j2: float, j3: float) -> Point3D:
    return forward_kinematics_points(j1, j2, j3).tool


def inverse_kinematics(x: float, y: float, z: float) -> dict[str, float]:
    radial = math.hypot(x, y)
    relative_z = z - BASE_HEIGHT_MM
    distance_sq = radial * radial + relative_z * relative_z
    min_reach = abs(UPPER_ARM_MM - FOREARM_MM)
    max_reach = UPPER_ARM_MM + FOREARM_MM
    distance = math.sqrt(distance_sq)
    if distance > max_reach + 1e-6 or distance < min_reach - 1e-6:
        raise PlanningError(f"目标超出 3-DOF 工作空间: x={x:.1f}, y={y:.1f}, z={z:.1f}")
    if z < MIN_TCP_Z_MM:
        raise PlanningError(f"目标低于基准平面: z={z:.1f}mm")

    yaw = 0.0 if radial < 1e-6 else math.degrees(math.atan2(y, x))
    cos_elbow = (distance_sq - UPPER_ARM_MM**2 - FOREARM_MM**2) / (2 * UPPER_ARM_MM * FOREARM_MM)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    candidates: list[dict[str, float]] = []
    for elbow_sign in (-1.0, 1.0):
        j3 = math.degrees(elbow_sign * math.acos(cos_elbow))
        elbow_rad = math.radians(j3)
        j2 = math.degrees(math.atan2(relative_z, radial) - math.atan2(FOREARM_MM * math.sin(elbow_rad), UPPER_ARM_MM + FOREARM_MM * math.cos(elbow_rad)))
        joints = {"j1": normalize_yaw(yaw), "j2": round(j2, 3), "j3": round(j3, 3)}
        try:
            validate_joint_pose(joints)
        except PlanningError:
            continue
        candidates.append(joints)
    if not candidates:
        raise PlanningError(f"目标姿态受关节限位约束不可达: x={x:.1f}, y={y:.1f}, z={z:.1f}")
    return min(candidates, key=lambda item: abs(item["j2"] - SAFE_YAW_JOINTS["j2"]) + abs(item["j3"] - SAFE_YAW_JOINTS["j3"]))


def validate_joint_limits(joints: dict[str, float]) -> None:
    if not J1_LIMIT_DEG[0] <= float(joints["j1"]) <= J1_LIMIT_DEG[1]:
        raise PlanningError(f"J1 超限: {joints['j1']}")
    if not J2_LIMIT_DEG[0] <= float(joints["j2"]) <= J2_LIMIT_DEG[1]:
        raise PlanningError(f"J2 超限: {joints['j2']}")
    if not J3_LIMIT_DEG[0] <= float(joints["j3"]) <= J3_LIMIT_DEG[1]:
        raise PlanningError(f"J3 超限: {joints['j3']}")


def validate_joint_pose(joints: dict[str, float]) -> None:
    validate_joint_limits(joints)
    point = forward_kinematics(float(joints["j1"]), float(joints["j2"]), float(joints["j3"]))
    if point.z < MIN_TCP_Z_MM:
        raise PlanningError(f"TCP 低于基准平面: z={point.z:.1f}mm")


def parse_distance_mm(text: str) -> int:
    match = re.search(r"(\d+(?:\.\d+)?)\s*(毫米|mm|厘米|公分|cm|米|m)", text, re.IGNORECASE)
    if not match:
        return 300
    value = float(match.group(1))
    unit = match.group(2).lower()
    if unit in {"米", "m"}:
        return round(value * 1000)
    if unit in {"厘米", "公分", "cm"}:
        return round(value * 10)
    return round(value)


def parse_direction_xy(text: str, distance: int) -> tuple[int, int]:
    cleaned = text.replace("然后", "").replace("之后", "").replace("随后", "")
    direction_patterns = [
        (r"左侧|左边|左方|左面|左", (0, distance)),
        (r"右侧|右边|右方|右面|右", (0, -distance)),
        (r"后面|后方|后侧|后边|向后|往后|(?<!然)后", (-distance, 0)),
        (r"前面|前方|前侧|前边|向前|往前|前", (distance, 0)),
    ]
    for pattern, point in direction_patterns:
        if re.search(pattern, cleaned):
            return point
    return distance, 0


def parse_target(text: str, default_z: float | None = None) -> Point3D:
    distance = parse_distance_mm(text)
    x, y = parse_direction_xy(text, distance)
    if any(word in text for word in ["桌面", "台面", "平台"]):
        z = TABLE_HEIGHT_MM
    elif "地面" in text:
        z = GRASP_HEIGHT_MM
    elif default_z is not None:
        z = default_z
    else:
        z = GRASP_HEIGHT_MM if "上" in text else PRE_GRASP_HEIGHT_MM
    return Point3D(float(x), float(y), float(z))


def target_from_spec(spec: dict[str, Any], default_z: float = GRASP_HEIGHT_MM) -> Point3D:
    if not isinstance(spec, dict):
        raise PlanningError("target 必须是对象")
    distance = spec.get("distance_mm")
    if distance is None and spec.get("distance_cm") is not None:
        distance = float(spec["distance_cm"]) * 10
    if distance is None and spec.get("distance_m") is not None:
        distance = float(spec["distance_m"]) * 1000
    distance = int(round(float(distance if distance is not None else 300)))

    direction = str(spec.get("direction", "front")).lower()
    if any(word in direction for word in ["左", "left"]):
        x, y = 0, distance
    elif any(word in direction for word in ["右", "right"]):
        x, y = 0, -distance
    elif any(word in direction for word in ["后", "back", "rear", "behind"]):
        x, y = -distance, 0
    else:
        x, y = distance, 0

    surface = str(spec.get("surface", "")).lower()
    if any(word in surface for word in ["桌", "台", "平台", "table", "desk", "platform"]):
        z = TABLE_HEIGHT_MM
    elif any(word in surface for word in ["地", "ground", "floor"]):
        z = GRASP_HEIGHT_MM
    elif spec.get("z_mm") is not None:
        z = float(spec["z_mm"])
    else:
        z = default_z
    return Point3D(float(x), float(y), float(z))


def find_first_keyword(text: str, keywords: list[str]) -> tuple[int, str] | None:
    matches = [(index, keyword) for keyword in keywords if (index := text.find(keyword)) >= 0]
    return min(matches, default=None)


def parse_pick_and_place_targets(description: str) -> tuple[Point3D, Point3D] | None:
    grasp_match = find_first_keyword(description, ["抓", "拿", "夹取", "拾取"])
    place_match = find_first_keyword(description, ["放到", "放置到", "放置", "放在", "放下"])
    if not grasp_match or not place_match or grasp_match[0] >= place_match[0]:
        return None
    source_text = description[: place_match[0]]
    destination_text = description[place_match[0] :]
    source = parse_target(source_text, default_z=GRASP_HEIGHT_MM)
    destination = parse_target(destination_text, default_z=GRASP_HEIGHT_MM)
    return source, destination


def describe_target(target: Point3D) -> dict[str, Any]:
    joints = inverse_kinematics(target.x, target.y, target.z)
    actual = forward_kinematics(joints["j1"], joints["j2"], joints["j3"])
    requested_radial = math.hypot(target.x, target.y)
    actual_radial = math.hypot(actual.x, actual.y)
    return {
        "requested_tcp": target.to_dict(),
        "planned_joints": {"j1": round(joints["j1"], 3), "j2": round(joints["j2"], 3), "j3": round(joints["j3"], 3)},
        "actual_tcp": actual.to_dict(),
        "requested_radial_mm": round(requested_radial, 3),
        "actual_radial_mm": round(actual_radial, 3),
        "radial_error_mm": round(actual_radial - requested_radial, 3),
        "note": "3-DOF yaw+shoulder+elbow 机构可用 J2/J3 同时调节高度与径向距离。",
    }


def build_metadata(description: str, target: Point3D | None, compound_targets: tuple[Point3D, Point3D] | None = None) -> dict[str, Any]:
    metadata: dict[str, Any] = {
        "model": "3-DOF yaw+shoulder+elbow",
        "base_height_mm": BASE_HEIGHT_MM,
        "upper_arm_mm": UPPER_ARM_MM,
        "forearm_mm": FOREARM_MM,
        "arm_reach_mm": ARM_REACH_MM,
        "home_joints": dict(HOME_JOINTS),
        "joint_limits_deg": {"j1": list(J1_LIMIT_DEG), "j2": list(J2_LIMIT_DEG), "j3": list(J3_LIMIT_DEG)},
    }
    if target is not None:
        metadata["target_analysis"] = describe_target(target)
    if compound_targets is not None:
        source, destination = compound_targets
        metadata["compound_target_analysis"] = {
            "source": describe_target(source),
            "destination": describe_target(destination),
        }
    return metadata


def interpolate_joints(start: dict[str, float], end: dict[str, float], segments: int) -> list[dict[str, float]]:
    if segments <= 1:
        return [end]
    yaw_delta = shortest_yaw_delta(float(start["j1"]), float(end["j1"]))
    j2_delta = float(end["j2"]) - float(start["j2"])
    j3_delta = float(end["j3"]) - float(start["j3"])
    result = []
    for index in range(1, segments + 1):
        ratio = index / segments
        smooth = ratio * ratio * (3 - 2 * ratio)
        result.append(
            {
                "j1": normalize_yaw(float(start["j1"]) + yaw_delta * smooth),
                "j2": round(float(start["j2"]) + j2_delta * smooth, 3),
                "j3": round(float(start["j3"]) + j3_delta * smooth, 3),
            }
        )
    return result


def make_move(step: int, comment: str, joints: dict[str, float], speed: str = "medium") -> dict[str, Any]:
    validate_joint_pose(joints)
    position = forward_kinematics(joints["j1"], joints["j2"], joints["j3"])
    return {
        "step": step,
        "action": "move_joints",
        "comment": comment,
        "params": {
            "joints": {"j1": round(float(joints["j1"]), 3), "j2": round(float(joints["j2"]), 3), "j3": round(float(joints["j3"]), 3)},
            "tcp": {"x": position.x, "y": position.y, "z": position.z},
            "speed": speed,
        },
    }


def make_gripper(step: int, state: str, width: float, force: float, comment: str) -> dict[str, Any]:
    return {"step": step, "action": "gripper", "comment": comment, "params": {"state": state, "width": width, "force": force}}


def make_wait(step: int, duration_ms: int, comment: str) -> dict[str, Any]:
    return {"step": step, "action": "wait", "comment": comment, "params": {"duration_ms": duration_ms}}


def safe_yaw_joints(j1: float) -> dict[str, float]:
    return {"j1": normalize_yaw(j1), "j2": SAFE_YAW_JOINTS["j2"], "j3": SAFE_YAW_JOINTS["j3"]}


def make_hover_point(target: Point3D) -> Point3D:
    desired_z = max(PRE_GRASP_HEIGHT_MM, target.z + APPROACH_CLEARANCE_MM)
    candidates = [desired_z, target.z + 80, target.z + 60, target.z + 40, target.z + 20, target.z]
    for z in sorted({round(value, 3) for value in candidates if value >= target.z}, reverse=True):
        hover = Point3D(target.x, target.y, float(z))
        try:
            inverse_kinematics(hover.x, hover.y, hover.z)
        except PlanningError:
            continue
        return hover
    raise PlanningError(f"无法为目标生成可达悬停点: x={target.x:.1f}, y={target.y:.1f}, z={target.z:.1f}")


def plan_grasp(description: str, target: Point3D, start_joints: dict[str, float] | None = None, return_home: bool = True) -> list[dict[str, Any]]:
    start = start_joints or dict(HOME_JOINTS)
    hover = make_hover_point(target)
    target_yaw = inverse_kinematics(hover.x, hover.y, hover.z)["j1"]
    start_safe_joints = safe_yaw_joints(start["j1"])
    target_safe_joints = safe_yaw_joints(target_yaw)
    hover_joints = inverse_kinematics(hover.x, hover.y, hover.z)
    target_joints = inverse_kinematics(target.x, target.y, target.z)

    steps: list[dict[str, Any]] = []
    step = 1
    steps.append(make_gripper(step, "open", 80, 0, "预开夹爪")); step += 1
    steps.append(make_move(step, "肩肘展开到安全转向姿态", start_safe_joints, "medium")); step += 1
    if abs(shortest_yaw_delta(start["j1"], target_yaw)) >= 0.5:
        steps.append(make_move(step, "保持肩肘姿态旋转基座对准目标方位", target_safe_joints, "medium")); step += 1
    steps.append(make_move(step, "肩肘协同伸出到目标上方", hover_joints, "medium")); step += 1
    steps.append(make_move(step, "保持方位下降到抓取点", target_joints, "slow")); step += 1
    steps.append(make_gripper(step, "close", 20, 50, "闭合夹爪抓取")); step += 1
    steps.append(make_wait(step, 500, "等待夹持稳定")); step += 1
    steps.append(make_move(step, "肩肘协同抬起目标物", hover_joints, "slow")); step += 1
    steps.append(make_move(step, "回到当前方位安全转向姿态", target_safe_joints, "medium")); step += 1
    if return_home:
        home_safe_joints = safe_yaw_joints(HOME_JOINTS["j1"])
        if abs(shortest_yaw_delta(target_safe_joints["j1"], HOME_JOINTS["j1"])) >= 0.5:
            steps.append(make_move(step, "保持肩肘姿态旋回 HOME 方位", home_safe_joints, "medium")); step += 1
        steps.append(make_move(step, "肩肘垂直收拢回 HOME", HOME_JOINTS, "medium")); step += 1
    return renumber_steps(steps)


def plan_place(description: str, target: Point3D, start_joints: dict[str, float] | None = None, return_home: bool = True) -> list[dict[str, Any]]:
    start = start_joints or dict(HOME_JOINTS)
    hover = make_hover_point(target)
    target_yaw = inverse_kinematics(hover.x, hover.y, hover.z)["j1"]
    start_safe_joints = safe_yaw_joints(start["j1"])
    target_safe_joints = safe_yaw_joints(target_yaw)
    hover_joints = inverse_kinematics(hover.x, hover.y, hover.z)
    target_joints = inverse_kinematics(target.x, target.y, target.z)

    steps: list[dict[str, Any]] = []
    step = 1
    steps.append(make_move(step, "肩肘展开到安全转向姿态", start_safe_joints, "medium")); step += 1
    if abs(shortest_yaw_delta(start["j1"], target_yaw)) >= 0.5:
        steps.append(make_move(step, "保持肩肘姿态旋转基座对准放置方位", target_safe_joints, "medium")); step += 1
    steps.append(make_move(step, "肩肘协同伸出到放置点上方", hover_joints, "medium")); step += 1
    steps.append(make_move(step, "保持方位下降到放置点", target_joints, "slow")); step += 1
    steps.append(make_gripper(step, "open", 80, 0, "打开夹爪释放")); step += 1
    steps.append(make_wait(step, 500, "等待物体稳定")); step += 1
    steps.append(make_move(step, "回到当前方位安全转向姿态", target_safe_joints, "medium")); step += 1
    if return_home:
        home_safe_joints = safe_yaw_joints(HOME_JOINTS["j1"])
        if abs(shortest_yaw_delta(target_safe_joints["j1"], HOME_JOINTS["j1"])) >= 0.5:
            steps.append(make_move(step, "保持肩肘姿态旋回 HOME 方位", home_safe_joints, "medium")); step += 1
        steps.append(make_move(step, "肩肘垂直收拢回 HOME", HOME_JOINTS, "medium")); step += 1
    return renumber_steps(steps)


def plan_pick_and_place(description: str, source: Point3D, destination: Point3D, start_joints: dict[str, float] | None = None, return_home: bool = True) -> list[dict[str, Any]]:
    start = start_joints or dict(HOME_JOINTS)
    source_hover = make_hover_point(source)
    destination_hover = make_hover_point(destination)

    source_yaw = inverse_kinematics(source_hover.x, source_hover.y, source_hover.z)["j1"]
    destination_yaw = inverse_kinematics(destination_hover.x, destination_hover.y, destination_hover.z)["j1"]
    start_safe_joints = safe_yaw_joints(start["j1"])
    source_safe_joints = safe_yaw_joints(source_yaw)
    destination_safe_joints = safe_yaw_joints(destination_yaw)
    source_hover_joints = inverse_kinematics(source_hover.x, source_hover.y, source_hover.z)
    source_joints = inverse_kinematics(source.x, source.y, source.z)
    destination_hover_joints = inverse_kinematics(destination_hover.x, destination_hover.y, destination_hover.z)
    destination_joints = inverse_kinematics(destination.x, destination.y, destination.z)

    steps: list[dict[str, Any]] = []
    step = 1
    steps.append(make_gripper(step, "open", 80, 0, "预开夹爪")); step += 1
    steps.append(make_move(step, "肩肘展开到安全转向姿态", start_safe_joints, "medium")); step += 1
    if abs(shortest_yaw_delta(start["j1"], source_yaw)) >= 0.5:
        steps.append(make_move(step, "保持肩肘姿态旋转基座对准抓取方位", source_safe_joints, "medium")); step += 1
    steps.append(make_move(step, "肩肘协同伸出到抓取点上方", source_hover_joints, "medium")); step += 1
    steps.append(make_move(step, "保持方位下降到抓取点", source_joints, "slow")); step += 1
    steps.append(make_gripper(step, "close", 20, 50, "闭合夹爪抓取")); step += 1
    steps.append(make_wait(step, 500, "等待夹持稳定")); step += 1
    steps.append(make_move(step, "肩肘协同抬起目标物", source_hover_joints, "slow")); step += 1
    steps.append(make_move(step, "回到当前方位安全转向姿态", source_safe_joints, "medium")); step += 1
    if abs(shortest_yaw_delta(source_yaw, destination_yaw)) >= 0.5:
        steps.append(make_move(step, "保持肩肘姿态旋转基座对准放置方位", destination_safe_joints, "medium")); step += 1
    steps.append(make_move(step, "肩肘协同伸出到放置点上方", destination_hover_joints, "medium")); step += 1
    steps.append(make_move(step, "保持方位下降到放置点", destination_joints, "slow")); step += 1
    steps.append(make_gripper(step, "open", 80, 0, "打开夹爪释放")); step += 1
    steps.append(make_wait(step, 500, "等待物体稳定")); step += 1
    steps.append(make_move(step, "肩肘协同抬离放置点", destination_hover_joints, "slow")); step += 1
    steps.append(make_move(step, "回到当前方位安全转向姿态", destination_safe_joints, "medium")); step += 1
    if return_home:
        home_safe_joints = safe_yaw_joints(HOME_JOINTS["j1"])
        if abs(shortest_yaw_delta(destination_yaw, HOME_JOINTS["j1"])) >= 0.5:
            steps.append(make_move(step, "保持肩肘姿态旋回 HOME 方位", home_safe_joints, "medium")); step += 1
        steps.append(make_move(step, "肩肘垂直收拢回 HOME", HOME_JOINTS, "medium")); step += 1
    return renumber_steps(steps)


def append_safe_move(steps: list[dict[str, Any]], step: int, current_joints: dict[str, float], speed: str = "medium") -> tuple[int, dict[str, float]]:
    safe_joints = safe_yaw_joints(current_joints["j1"])
    if abs(current_joints["j2"] - safe_joints["j2"]) < 1e-6 and abs(current_joints["j3"] - safe_joints["j3"]) < 1e-6:
        return step, current_joints
    comment = "肩肘展开到安全转向姿态" if abs(shortest_yaw_delta(current_joints["j1"], HOME_JOINTS["j1"])) < 1e-6 else "回到当前方位安全转向姿态"
    steps.append(make_move(step, comment, safe_joints, speed))
    return step + 1, safe_joints


def append_approach_target(
    steps: list[dict[str, Any]],
    step: int,
    current_joints: dict[str, float],
    target: Point3D,
    label: str,
) -> tuple[int, dict[str, float], dict[str, float], dict[str, float]]:
    hover = make_hover_point(target)
    target_yaw = inverse_kinematics(hover.x, hover.y, hover.z)["j1"]
    target_safe_joints = safe_yaw_joints(target_yaw)
    hover_joints = inverse_kinematics(hover.x, hover.y, hover.z)
    target_joints = inverse_kinematics(target.x, target.y, target.z)

    step, current_joints = append_safe_move(steps, step, current_joints)
    if abs(shortest_yaw_delta(current_joints["j1"], target_yaw)) >= 0.5:
        steps.append(make_move(step, f"保持肩肘姿态旋转基座对准{label}方位", target_safe_joints, "medium"))
        step += 1
        current_joints = target_safe_joints
    steps.append(make_move(step, f"肩肘协同伸出到{label}点上方", hover_joints, "medium")); step += 1
    steps.append(make_move(step, f"保持方位下降到{label}点", target_joints, "slow")); step += 1
    return step, target_joints, hover_joints, target_safe_joints


def append_return_home(steps: list[dict[str, Any]], step: int, current_joints: dict[str, float]) -> tuple[int, dict[str, float]]:
    step, current_joints = append_safe_move(steps, step, current_joints)
    home_safe_joints = safe_yaw_joints(HOME_JOINTS["j1"])
    if abs(shortest_yaw_delta(current_joints["j1"], HOME_JOINTS["j1"])) >= 0.5:
        steps.append(make_move(step, "保持肩肘姿态旋回 HOME 方位", home_safe_joints, "medium"))
        step += 1
        current_joints = home_safe_joints
    steps.append(make_move(step, "肩肘垂直收拢回 HOME", HOME_JOINTS, "medium"))
    return step + 1, dict(HOME_JOINTS)


def normalize_command_type(raw_type: Any) -> str:
    value = str(raw_type or "").strip().lower()
    if value in {"pick", "grasp", "grab", "pickup", "抓取", "拿取", "拾取"}:
        return "pick"
    if value in {"place", "put", "drop", "release", "放置", "放下"}:
        return "place"
    if value in {"home", "reset", "复位", "回零"}:
        return "home"
    raise PlanningError(f"不支持的语义命令类型: {raw_type}")


def command_target(command: dict[str, Any], default_z: float) -> Point3D:
    target = command.get("target")
    if target is None:
        raise PlanningError(f"{command.get('type')} 命令缺少 target")
    return target_from_spec(target, default_z=default_z)


def plan_command_sequence(commands: list[dict[str, Any]], start_joints: dict[str, float] | None = None, start_gripper: str = "open", return_home: bool = True) -> list[dict[str, Any]]:
    if not commands:
        raise PlanningError("语义命令序列不能为空")

    steps: list[dict[str, Any]] = []
    step = 1
    current_joints = dict(start_joints) if start_joints else dict(HOME_JOINTS)
    gripper_state = start_gripper

    for command in commands:
        command_type = normalize_command_type(command.get("type"))
        if command_type == "home":
            step, current_joints = append_return_home(steps, step, current_joints)
            continue

        if command_type == "pick":
            target = command_target(command, default_z=GRASP_HEIGHT_MM)
            if gripper_state != "open":
                steps.append(make_gripper(step, "open", 80, 0, "打开夹爪准备抓取")); step += 1
                gripper_state = "open"
            elif not steps:
                steps.append(make_gripper(step, "open", 80, 0, "预开夹爪")); step += 1
            step, current_joints, hover_joints, target_safe_joints = append_approach_target(steps, step, current_joints, target, "抓取")
            steps.append(make_gripper(step, "close", 20, 50, "闭合夹爪抓取")); step += 1
            gripper_state = "close"
            steps.append(make_wait(step, 500, "等待夹持稳定")); step += 1
            steps.append(make_move(step, "肩肘协同抬起目标物", hover_joints, "slow")); step += 1
            steps.append(make_move(step, "回到当前方位安全转向姿态", target_safe_joints, "medium")); step += 1
            current_joints = target_safe_joints
            continue

        if command_type == "place":
            target = command_target(command, default_z=GRASP_HEIGHT_MM)
            step, current_joints, hover_joints, target_safe_joints = append_approach_target(steps, step, current_joints, target, "放置")
            steps.append(make_gripper(step, "open", 80, 0, "打开夹爪释放")); step += 1
            gripper_state = "open"
            steps.append(make_wait(step, 500, "等待物体稳定")); step += 1
            steps.append(make_move(step, "肩肘协同抬离放置点", hover_joints, "slow")); step += 1
            steps.append(make_move(step, "回到当前方位安全转向姿态", target_safe_joints, "medium")); step += 1
            current_joints = target_safe_joints

    if return_home:
        if not steps or steps[-1].get("action") != "move_joints" or steps[-1]["params"]["joints"] != HOME_JOINTS:
            append_return_home(steps, step, current_joints)
    return renumber_steps(steps)


def build_task_from_command_plan(plan: dict[str, Any], original_description: str | None = None, start_joints: dict[str, float] | None = None, start_gripper: str = "open", return_home: bool = True) -> dict[str, Any]:
    commands = plan.get("commands")
    if not isinstance(commands, list):
        raise PlanningError("语义 JSON 缺少 commands 数组")
    steps = plan_command_sequence(commands, start_joints=start_joints, start_gripper=start_gripper, return_home=return_home)
    task = {
        "task_id": str(uuid.uuid4()),
        "task_description": original_description or str(plan.get("description") or "结构化语义命令"),
        "created_at": datetime.now(CHINA_TZ).isoformat(timespec="seconds"),
        "metadata": {
            **build_metadata(original_description or "", None),
            "semantic_plan": plan,
            "command_target_analysis": [
                {"type": normalize_command_type(command.get("type")), "target": describe_target(command_target(command, GRASP_HEIGHT_MM))}
                for command in commands
                if normalize_command_type(command.get("type")) in {"pick", "place"}
            ],
        },
        "steps": steps,
    }
    validate_task(task, start_joints=start_joints)
    return task


def plan_home(start_joints: dict[str, float] | None = None) -> list[dict[str, Any]]:
    start = start_joints or dict(HOME_JOINTS)
    steps: list[dict[str, Any]] = []
    append_return_home(steps, 1, start)
    if not steps:
        steps.append(make_move(1, "回到三关节 HOME 姿态", HOME_JOINTS, "medium"))
    return renumber_steps(steps)


def renumber_steps(steps: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for index, item in enumerate(steps, start=1):
        item["step"] = index
    return steps


def build_task(description: str, start_joints: dict[str, float] | None = None, return_home: bool = True) -> dict[str, Any]:
    if not description.strip():
        raise PlanningError("任务描述不能为空")
    target: Point3D | None = None
    compound_targets: tuple[Point3D, Point3D] | None = None
    if any(word in description for word in ["回零", "复位", "回到初始", "回家", "home"]):
        steps = plan_home(start_joints=start_joints)
    elif compound_targets := parse_pick_and_place_targets(description):
        source, destination = compound_targets
        steps = plan_pick_and_place(description, source, destination, start_joints=start_joints, return_home=return_home)
    elif any(word in description for word in ["放到", "放置", "放下"]):
        target = parse_target(description, default_z=GRASP_HEIGHT_MM)
        steps = plan_place(description, target, start_joints=start_joints, return_home=return_home)
    elif any(word in description for word in ["抓", "拿", "夹取", "拾取"]):
        target = parse_target(description, default_z=GRASP_HEIGHT_MM)
        steps = plan_grasp(description, target, start_joints=start_joints, return_home=return_home)
    else:
        raise PlanningError("暂只支持抓取、放置和复位任务")
    task = {
        "task_id": str(uuid.uuid4()),
        "task_description": description,
        "created_at": datetime.now(CHINA_TZ).isoformat(timespec="seconds"),
        "metadata": build_metadata(description, target, compound_targets),
        "steps": steps,
    }
    validate_task(task, start_joints=start_joints)
    return task


def validate_task(task: dict[str, Any], start_joints: dict[str, float] | None = None) -> None:
    for field in ["task_id", "task_description", "created_at", "steps"]:
        if field not in task:
            raise PlanningError(f"任务缺少字段: {field}")
    if not task["steps"]:
        raise PlanningError("任务至少需要一个步骤")
    current_joints = dict(start_joints) if start_joints else dict(HOME_JOINTS)
    target_yaw: float | None = None
    for expected, step in enumerate(task["steps"], start=1):
        if step.get("step") != expected:
            raise PlanningError("步骤编号必须连续")
        validate_step(step)
        if step.get("action") == "move_joints":
            target_joints = {name: float(step["params"]["joints"][name]) for name in ["j1", "j2", "j3"]}
            comment = step.get("comment", "")
            if "旋转基座对准" in comment:
                if abs(target_joints["j2"] - current_joints["j2"]) > 1e-6 or abs(target_joints["j3"] - current_joints["j3"]) > 1e-6:
                    raise PlanningError("基座对准阶段只能改变 J1，不能改变 J2/J3")
                target_yaw = target_joints["j1"]
            elif target_yaw is not None and "收回" not in comment and "旋回" not in comment and "HOME" not in comment:
                if abs(shortest_yaw_delta(target_yaw, target_joints["j1"])) > 1e-3:
                    raise PlanningError("下降/夹取阶段必须保持已对准的 J1 方位")
            for sampled in interpolate_joints(current_joints, target_joints, 32):
                validate_joint_pose(sampled)
            current_joints = target_joints


def validate_step(step: dict[str, Any]) -> None:
    action = step.get("action")
    params = step.get("params", {})
    if action == "move_joints":
        joints = params.get("joints", {})
        validate_joint_pose({"j1": float(joints["j1"]), "j2": float(joints["j2"]), "j3": float(joints["j3"])})
        tcp = params.get("tcp", {})
        if "z" in tcp and float(tcp["z"]) < MIN_TCP_Z_MM:
            raise PlanningError("move_joints.tcp.z 不能小于 0")
        if params.get("speed") not in {"slow", "medium", "fast"}:
            raise PlanningError("move_joints.speed 必须是 slow/medium/fast")
    elif action == "gripper":
        if params.get("state") not in {"open", "close"}:
            raise PlanningError("gripper.state 必须是 open/close")
        if not 0 <= float(params.get("width", -1)) <= 100:
            raise PlanningError("gripper.width 必须在 0-100mm")
        if not 0 <= float(params.get("force", -1)) <= 100:
            raise PlanningError("gripper.force 必须在 0-100")
    elif action == "wait":
        if int(params.get("duration_ms", -1)) < 0:
            raise PlanningError("wait.duration_ms 不能为负数")
    else:
        raise PlanningError(f"不支持的 action: {action}")


def extract_end_state(task: dict[str, Any], start_joints: dict[str, float] | None = None, start_gripper: dict[str, float | str] | None = None) -> tuple[dict[str, float], dict[str, float | str]]:
    """从已完成任务中提取最终关节角和夹爪状态。"""
    joints = dict(start_joints) if start_joints else dict(HOME_JOINTS)
    gripper: dict[str, float | str] = dict(start_gripper) if start_gripper else {"state": "open", "width": 100.0, "force": 0.0}
    for step in task.get("steps", []):
        if step["action"] == "move_joints":
            joints = {k: float(step["params"]["joints"][k]) for k in ["j1", "j2", "j3"]}
        elif step["action"] == "gripper":
            gripper = {"state": step["params"]["state"], "width": float(step["params"]["width"]), "force": float(step["params"]["force"])}
    return joints, gripper


def main() -> int:
    configure_stdio()
    parser = argparse.ArgumentParser(description="用本地规则生成 3-DOF 机械臂可执行任务 JSON")
    parser.add_argument("description")
    parser.add_argument("--compact", action="store_true")
    parser.add_argument("--no-return-home", action="store_true", help="执行完毕后不回 HOME")
    args = parser.parse_args()
    try:
        task = build_task(args.description, return_home=not args.no_return_home)
    except PlanningError as exc:
        print(f"规划失败: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(task, ensure_ascii=False, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
