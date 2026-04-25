"""Web 后端与动画帧生成模块。

负责提供静态页面服务、`/api/random`、`/api/workflow` 和 `/api/reset` 接口，
并将已校验任务 JSON 转换为前端动画关键帧。服务端维护机械臂当前关节角和
夹爪状态（线程安全），每次 `/api/workflow` 从当前状态起始规划，完成后更新
状态。`/api/reset` 可将状态重置为 HOME。`/api/workflow` 接收
`task_text/model/retries/local_first/return_home`，默认 `return_home=false`，
再把任务 JSON 和动画帧交给 `web/app.js` 渲染。
"""

import argparse
import json
import random
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from arm_planner import GRIPPER_DURATION_MS, HOME_JOINTS, MOVE_DURATION_MS, extract_end_state, forward_kinematics, forward_kinematics_points, validate_task
from llm_planner import LLMPlanningError, build_task_with_llm
from random_task_generator import generate_task


ROOT = Path(__file__).resolve().parent
WEB_DIR = ROOT / "web"

_arm_lock = threading.Lock()
_arm_state: dict[str, Any] = {
    "joints": dict(HOME_JOINTS),
    "gripper": {"state": "open", "width": 100.0, "force": 0.0},
}


def make_frame(step: int, action: str, comment: str, joints: dict[str, float], gripper: dict[str, float | str], elapsed_ms: int) -> dict[str, Any]:
    tcp = forward_kinematics(joints["j1"], joints["j2"], joints["j3"])
    points = forward_kinematics_points(joints["j1"], joints["j2"], joints["j3"])
    return {
        "step": step,
        "action": action,
        "comment": comment,
        "joints": dict(joints),
        "position": {"x": tcp.x, "y": tcp.y, "z": tcp.z},
        "points": {
            "base": points.base.to_dict(),
            "shoulder": points.shoulder.to_dict(),
            "elbow": points.elbow.to_dict(),
            "tool": points.tool.to_dict(),
        },
        "gripper": dict(gripper),
        "elapsed_ms": elapsed_ms,
    }


def build_frames(task: dict[str, Any], start_joints: dict[str, float] | None = None, start_gripper: dict[str, float | str] | None = None) -> list[dict[str, Any]]:
    validate_task(task, start_joints=start_joints)
    joints = dict(start_joints) if start_joints else dict(HOME_JOINTS)
    gripper: dict[str, float | str] = dict(start_gripper) if start_gripper else {"state": "open", "width": 100.0, "force": 0.0}
    elapsed_ms = 0
    frames = [make_frame(0, "init", "初始姿态", joints, gripper, elapsed_ms)]

    for step in task["steps"]:
        action = step["action"]
        params = step["params"]
        comment = step.get("comment", "")
        if action == "move_joints":
            joints = {"j1": float(params["joints"]["j1"]), "j2": float(params["joints"]["j2"]), "j3": float(params["joints"]["j3"])}
            elapsed_ms += MOVE_DURATION_MS.get(params.get("speed", "medium"), MOVE_DURATION_MS["medium"])
            frames.append(make_frame(step["step"], action, comment, joints, gripper, elapsed_ms))
        elif action == "gripper":
            gripper = {"state": params["state"], "width": float(params["width"]), "force": float(params["force"])}
            elapsed_ms += GRIPPER_DURATION_MS
            frames.append(make_frame(step["step"], action, comment, joints, gripper, elapsed_ms))
        elif action == "wait":
            elapsed_ms += int(params["duration_ms"])
            frames.append(make_frame(step["step"], action, comment, joints, gripper, elapsed_ms))
    return frames


def plan_with_retries(task_text: str, model: str | None, retries: int, local_first: bool, start_joints: dict[str, float] | None = None, start_gripper: str = "open", return_home: bool = True) -> dict[str, Any]:
    last_error: Exception | None = None
    for _ in range(max(retries, 1)):
        try:
            return build_task_with_llm(task_text, model=model, local_first=local_first, start_joints=start_joints, start_gripper=start_gripper, return_home=return_home)
        except LLMPlanningError as exc:
            last_error = exc
    raise RuntimeError(str(last_error) if last_error else "LLM planning failed")


def run_workflow(payload: dict[str, Any]) -> dict[str, Any]:
    task_text = str(payload.get("task_text", "")).strip()
    if not task_text:
        raise ValueError("请先输入自然语言任务，或点击随机生成填入任务")
    with _arm_lock:
        cur_joints = dict(_arm_state["joints"])
        cur_gripper = dict(_arm_state["gripper"])
    task = plan_with_retries(
        task_text,
        payload.get("model") or None,
        int(payload.get("retries", 3)),
        bool(payload.get("local_first", False)),
        start_joints=cur_joints,
        start_gripper=str(cur_gripper["state"]),
        return_home=bool(payload.get("return_home", False)),
    )
    frames = build_frames(task, start_joints=cur_joints, start_gripper=cur_gripper)
    end_joints, end_gripper = extract_end_state(task, start_joints=cur_joints, start_gripper=cur_gripper)
    with _arm_lock:
        _arm_state["joints"] = end_joints
        _arm_state["gripper"] = end_gripper
    return {"task_text": task_text, "task": task, "frames": frames}


class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        route = urlparse(self.path).path
        if route == "/":
            self.send_file(WEB_DIR / "index.html", "text/html; charset=utf-8")
        elif route == "/app.js":
            self.send_file(WEB_DIR / "app.js", "application/javascript; charset=utf-8")
        elif route == "/styles.css":
            self.send_file(WEB_DIR / "styles.css", "text/css; charset=utf-8")
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            payload = self.read_json()
            route = urlparse(self.path).path
            if route == "/api/random":
                rng = random.Random(int(payload["seed"])) if payload.get("seed") is not None else None
                self.send_json({"task_text": generate_task(payload.get("type", "mixed"), rng=rng)})
                return
            if route == "/api/workflow":
                self.send_json(run_workflow(payload))
                return
            if route == "/api/reset":
                with _arm_lock:
                    _arm_state["joints"] = dict(HOME_JOINTS)
                    _arm_state["gripper"] = {"state": "open", "width": 100.0, "force": 0.0}
                self.send_json({"status": "ok", "joints": dict(HOME_JOINTS)})
                return
            self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self.send_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        return json.loads(self.rfile.read(length).decode("utf-8")) if length else {}

    def send_file(self, path: Path, content_type: str) -> None:
        content = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format: str, *args: Any) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser(description="3-DOF 机械臂工作流 Web 界面")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), WebHandler)
    print(f"Web UI: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
