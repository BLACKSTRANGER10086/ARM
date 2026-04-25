"""自然语言任务的结构化语义解析模块。

负责优先调用 LLM 将用户输入整理成本地规划器可处理的结构化 JSON 命令；
LLM 不可用或语义命令不可规划时再回退本地规则。主要对外接口是
`build_task_with_llm()`。本模块不生成关节轨迹，而是调用
`arm_planner.build_task_from_command_plan()` 或本地回退入口 `build_task()`
生成最终任务 JSON，并被命令行入口、`workflow_demo.py` 和 `web_ui.py` 使用。
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

from arm_planner import PlanningError, build_task, build_task_from_command_plan, validate_task


ROOT = Path(__file__).resolve().parent
ENV_PATH = ROOT / ".env"
DEFAULT_MODEL = "deepseek-v4-flash"
LOCAL_FIRST_ENV = "ARM_LOCAL_FIRST"


class LLMPlanningError(RuntimeError):
    pass


def can_try_local_first(description: str) -> bool:
    if any(word in description for word in ["回零", "复位", "回到初始", "回家", "home"]):
        return True
    has_action = any(word in description for word in ["抓", "拿", "夹取", "拾取", "放到", "放置", "放下"])
    has_numeric_distance = re.search(r"\d+(?:\.\d+)?\s*(毫米|mm|厘米|公分|cm|米|m)", description, re.IGNORECASE)
    return has_action and bool(has_numeric_distance)


def env_flag(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def build_validated_task(planning_description: str, original_description: str, start_joints: dict[str, float] | None = None, return_home: bool = True) -> dict[str, Any]:
    task = build_task(planning_description, start_joints=start_joints, return_home=return_home)
    task["task_description"] = original_description
    validate_task(task)
    return task


def load_dotenv(path: Path = ENV_PATH) -> None:
    if not path.exists():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, value = stripped.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def create_client() -> Any:
    try:
        from openai import OpenAI
    except ImportError as exc:
        raise LLMPlanningError("缺少 openai 依赖，请运行 pip install -r requirements.txt") from exc
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise LLMPlanningError("未设置 OPENAI_API_KEY")
    base_url = os.getenv("OPENAI_BASE_URL")
    return OpenAI(api_key=api_key, base_url=base_url) if base_url else OpenAI(api_key=api_key)


def schema() -> dict[str, Any]:
    return {
        "type": "object",
        "additionalProperties": False,
        "required": ["commands"],
        "properties": {
            "description": {
                "type": "string",
                "description": "结构化命令对应的中文任务描述。",
            },
            "commands": {
                "type": "array",
                "minItems": 1,
                "items": {
                    "type": "object",
                    "additionalProperties": False,
                    "required": ["type"],
                    "properties": {
                        "type": {"type": "string", "enum": ["pick", "place", "home"]},
                        "target": {
                            "type": "object",
                            "additionalProperties": False,
                            "properties": {
                                "direction": {"type": "string", "description": "front/back/left/right 或中文方向"},
                                "distance_mm": {"type": "number"},
                                "distance_cm": {"type": "number"},
                                "surface": {"type": "string", "description": "ground/table/platform 或中文表面"},
                                "object": {"type": "string"},
                            },
                        },
                    },
                },
            },
        },
    }


def call_llm(description: str, model: str) -> str:
    client = create_client()
    prompt = (
        "你是3自由度机械臂的自然语言解析器。\n"
        "机械臂有 J1 底座 yaw、J2 肩部 pitch、J3 肘部 pitch：J1 决定前/后/左/右方位，J2/J3 协同决定高度与径向伸出；没有腕部自由度。\n"
        "请把用户指令转换为结构化 JSON 语义命令，不要输出关节角、轨迹点或 move_joints。\n"
        "commands 是顺序执行的高层命令数组，type 只能是 pick、place、home。\n"
        "pick/place 必须包含 target；target.direction 用 front/back/left/right，distance_mm 用毫米，surface 用 ground/table/platform，object 保留目标物名称。\n"
        "如果用户没有明确表面，抓取默认 ground，放置默认 ground；如果没有明确距离，可以根据语义合理补全常见距离。\n"
        "示例：'拿一下前面50厘米的盒子' -> commands=[{type:'pick', target:{direction:'front', distance_mm:500, surface:'ground', object:'盒子'}}]。\n"
        "示例：'把右侧20厘米地上的杯子抓起来放到前面5cm' -> commands=[{type:'pick', target:{direction:'right', distance_mm:200, surface:'ground', object:'杯子'}}, {type:'place', target:{direction:'front', distance_mm:50, surface:'ground'}}]。\n"
        f"用户指令：{description}"
    )
    try:
        response = client.responses.create(
            model=model,
            input=prompt,
            text={"format": {"type": "json_schema", "name": "three_dof_parse", "schema": schema(), "strict": False}},
        )
    except Exception as exc:
        raise LLMPlanningError(f"调用大模型失败: {exc}") from exc
    return response.output_text


def extract_json(text: str) -> dict[str, Any]:
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1:
        raise LLMPlanningError("模型未返回 JSON")
    return json.loads(text[start : end + 1])


def build_task_with_llm(description: str, model: str | None = None, debug: bool = False, local_first: bool | None = None, start_joints: dict[str, float] | None = None, start_gripper: str = "open", return_home: bool = True) -> dict[str, Any]:
    load_dotenv()
    use_local_first = env_flag(LOCAL_FIRST_ENV) if local_first is None else local_first
    if use_local_first and can_try_local_first(description):
        try:
            return build_validated_task(description, description, start_joints=start_joints, return_home=return_home)
        except PlanningError as exc:
            if debug:
                print(f"本地直接规划失败，尝试 LLM 结构化语义解析: {exc}", file=sys.stderr)

    selected_model = model or os.getenv("OPENAI_MODEL") or DEFAULT_MODEL
    try:
        parsed = extract_json(call_llm(description, selected_model))
        if "commands" in parsed:
            return build_task_from_command_plan(parsed, description, start_joints=start_joints, start_gripper=start_gripper, return_home=return_home)
        normalized = parsed.get("normalized_instruction") or parsed.get("description") or description
    except Exception as exc:
        if debug:
            print(f"LLM 结构化语义解析失败，回退本地规则: {exc}", file=sys.stderr)
        normalized = description
    try:
        return build_validated_task(normalized, description, start_joints=start_joints, return_home=return_home)
    except PlanningError as normalized_exc:
        if normalized != description:
            try:
                return build_validated_task(description, description, start_joints=start_joints, return_home=return_home)
            except PlanningError as original_exc:
                raise LLMPlanningError(f"3-DOF 规划失败: {normalized_exc}; 原始输入回退也失败: {original_exc}") from original_exc
        raise LLMPlanningError(f"3-DOF 规划失败: {normalized_exc}") from normalized_exc


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="调用 LLM 生成语义 commands，并由本地规划器生成 3-DOF 任务 JSON")
    parser.add_argument("description")
    parser.add_argument("--model", default=None)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--compact", action="store_true")
    parser.add_argument("--local-first", action="store_true", help="跳过 LLM，优先尝试本地中文规则解析")
    parser.add_argument("--no-return-home", action="store_true", help="执行完毕后不回 HOME")
    args = parser.parse_args()
    try:
        task = build_task_with_llm(args.description, args.model, args.debug, args.local_first, return_home=not args.no_return_home)
    except LLMPlanningError as exc:
        print(f"LLM 规划失败: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(task, ensure_ascii=False, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
