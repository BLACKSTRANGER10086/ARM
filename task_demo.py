"""终端版任务轨迹演示模块。

负责读取本地可执行任务 JSON，校验后将语义命令目标、关节运动、TCP、夹爪
状态和耗时打印为可读文本。主要对外接口是 `simulate_task()`。本模块依赖
`arm_planner.validate_task()` 和正运动学计算，可接收命令行管道输入，也被
`workflow_demo.py` 调用。
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

from arm_planner import ARM_REACH_MM, BASE_HEIGHT_MM, FOREARM_MM, GRIPPER_DURATION_MS, HOME_JOINTS, MOVE_DURATION_MS, UPPER_ARM_MM, forward_kinematics, validate_task


def load_task(path: str | None) -> dict[str, Any]:
    text = Path(path).read_text(encoding="utf-8") if path and path != "-" else sys.stdin.read()
    text = text.strip().lstrip("\ufeff")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start == -1 or end <= start:
            raise
        return json.loads(text[start : end + 1])


def joint_distance(a: dict[str, float], b: dict[str, float]) -> float:
    yaw_delta = ((b["j1"] - a["j1"] + 180) % 360) - 180
    return math.sqrt(yaw_delta**2 + (b["j2"] - a["j2"]) ** 2 + (b["j3"] - a["j3"]) ** 2)


def simulate_task(task: dict[str, Any], start_joints: dict[str, float] | None = None) -> str:
    validate_task(task, start_joints=start_joints)
    joints = dict(start_joints) if start_joints else dict(HOME_JOINTS)
    gripper: dict[str, float | str] = {"state": "open", "width": 100.0, "force": 0.0}
    elapsed_ms = 0
    lines = [
        f"Task: {task['task_description']}",
        f"Task ID: {task['task_id']}",
        "Model: 3-DOF arm (J1=base yaw, J2=shoulder pitch, J3=elbow pitch)",
        f"Geometry: base={BASE_HEIGHT_MM}mm, upper={UPPER_ARM_MM}mm, forearm={FOREARM_MM}mm, reach={ARM_REACH_MM}mm",
        f"Steps: {len(task['steps'])}",
        "",
    ]
    command_analysis = task.get("metadata", {}).get("command_target_analysis")
    compound_analysis = task.get("metadata", {}).get("compound_target_analysis")
    target_analysis = task.get("metadata", {}).get("target_analysis")
    if command_analysis:
        lines.append("Command Target Analysis:")
        for index, item in enumerate(command_analysis, start=1):
            requested = item["target"]["requested_tcp"]
            lines.append(f"  {index}. {item['type']} tcp=({requested['x']:.0f},{requested['y']:.0f},{requested['z']:.0f})mm")
        lines.append("")
    elif compound_analysis:
        source = compound_analysis["source"]["requested_tcp"]
        destination = compound_analysis["destination"]["requested_tcp"]
        lines.extend(
            [
                "Compound Target Analysis:",
                f"  source_tcp=({source['x']:.0f},{source['y']:.0f},{source['z']:.0f})mm",
                f"  destination_tcp=({destination['x']:.0f},{destination['y']:.0f},{destination['z']:.0f})mm",
                "",
            ]
        )
    elif target_analysis:
        requested = target_analysis["requested_tcp"]
        actual = target_analysis["actual_tcp"]
        lines.extend(
            [
                "Target Analysis:",
                f"  requested_tcp=({requested['x']:.0f},{requested['y']:.0f},{requested['z']:.0f})mm",
                f"  actual_tcp=({actual['x']:.0f},{actual['y']:.0f},{actual['z']:.0f})mm  radial_error={target_analysis['radial_error_mm']:.0f}mm",
                f"  note={target_analysis['note']}",
                "",
            ]
        )

    for step in task["steps"]:
        action = step["action"]
        params = step["params"]
        comment = step.get("comment", "")
        if action == "move_joints":
            previous = dict(joints)
            joints = {"j1": float(params["joints"]["j1"]), "j2": float(params["joints"]["j2"]), "j3": float(params["joints"]["j3"])}
            tcp = forward_kinematics(joints["j1"], joints["j2"], joints["j3"])
            lines.extend(
                [
                    f"[{elapsed_ms / 1000:.1f}s] Step {step['step']} MOVE_JOINTS  {comment}",
                    f"  J1/yaw {previous['j1']:.1f}° -> {joints['j1']:.1f}°  J2/shoulder {previous['j2']:.1f}° -> {joints['j2']:.1f}°  J3/elbow {previous['j3']:.1f}° -> {joints['j3']:.1f}°",
                    f"  joint_delta={joint_distance(previous, joints):.1f}°  speed={params['speed']}  tcp=({tcp.x:.0f},{tcp.y:.0f},{tcp.z:.0f})mm",
                    "",
                ]
            )
            elapsed_ms += MOVE_DURATION_MS.get(params.get("speed", "medium"), MOVE_DURATION_MS["medium"])
        elif action == "gripper":
            gripper = {"state": params["state"], "width": float(params["width"]), "force": float(params["force"])}
            lines.extend([f"[{elapsed_ms / 1000:.1f}s] Step {step['step']} GRIPPER  {comment}", f"  state={gripper['state']}  width={gripper['width']:.0f}mm  force={gripper['force']:.0f}", ""])
            elapsed_ms += GRIPPER_DURATION_MS
        elif action == "wait":
            duration_ms = int(params["duration_ms"])
            lines.extend([f"[{elapsed_ms / 1000:.1f}s] Step {step['step']} WAIT  {comment}", f"  duration={duration_ms}ms", ""])
            elapsed_ms += duration_ms

    tcp = forward_kinematics(joints["j1"], joints["j2"], joints["j3"])
    lines.extend(["Final State:", f"  joints=(yaw={joints['j1']:.1f}°, shoulder={joints['j2']:.1f}°, elbow={joints['j3']:.1f}°)", f"  tcp=({tcp.x:.0f},{tcp.y:.0f},{tcp.z:.0f})mm", f"  gripper={gripper['state']} width={float(gripper['width']):.0f}mm", f"  total_time={elapsed_ms / 1000:.1f}s"])
    return "\n".join(lines)


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="演示 3-DOF 机械臂可执行任务 JSON")
    parser.add_argument("json_path", nargs="?", default="-", help="任务 JSON 路径；默认从 stdin 读取")
    args = parser.parse_args()
    try:
        print(simulate_task(load_task(args.json_path)))
    except Exception as exc:
        print(f"演示失败: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
