"""自然语言任务的语义归一化模块。

负责优先调用 LLM 将用户输入整理成本地规划器可处理的中文任务；LLM 不可用
或归一化结果不可规划时再回退本地规则。主要对外接口是
`build_task_with_llm()`。本模块不生成关节轨迹，而是调用
`arm_planner.build_task()` 生成最终任务 JSON，并被命令行入口、
`workflow_demo.py` 和 `web_ui.py` 使用。
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

from arm_planner import PlanningError, build_task, validate_task


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


def build_validated_task(planning_description: str, original_description: str) -> dict[str, Any]:
    task = build_task(planning_description)
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
        "required": ["normalized_instruction"],
        "properties": {
            "normalized_instruction": {
                "type": "string",
                "description": "保留用户意图，明确抓取/放置/复位/抓取后放置、源位置、目标位置、地面/桌面位置的中文指令。",
            }
        },
    }


def call_llm(description: str, model: str) -> str:
    client = create_client()
    prompt = (
        "你是3自由度机械臂的自然语言解析器。\n"
        "机械臂有 J1 底座 yaw、J2 肩部 pitch、J3 肘部 pitch：J1 决定前/后/左/右方位，J2/J3 协同决定高度与径向伸出；没有腕部自由度。\n"
        "请把用户指令改写为一句清晰中文规范指令，必须保留任务类型、方向、距离和表面高度。\n"
        "支持任务类型：抓取、放置、复位、先抓取再放置。\n"
        "复合任务必须同时保留抓取源位置和放置目标位置，格式类似：先抓取右侧20厘米处地面上的杯子，然后放到前面5厘米处桌面上。\n"
        "示例：'拿一下前面50厘米的盒子' -> '抓取前面50厘米处地面上的盒子'。\n"
        "示例：'把它放左边三十公分' -> '放到左侧30厘米处台面上'。\n"
        "示例：'把右侧20厘米地上的杯子抓起来放到前面5cm' -> '先抓取右侧20厘米处地面上的杯子，然后放到前面5厘米处'。\n"
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


def build_task_with_llm(description: str, model: str | None = None, debug: bool = False, local_first: bool | None = None) -> dict[str, Any]:
    load_dotenv()
    use_local_first = env_flag(LOCAL_FIRST_ENV) if local_first is None else local_first
    if use_local_first and can_try_local_first(description):
        try:
            return build_validated_task(description, description)
        except PlanningError as exc:
            if debug:
                print(f"本地直接规划失败，尝试 LLM 归一化: {exc}", file=sys.stderr)

    selected_model = model or os.getenv("OPENAI_MODEL") or DEFAULT_MODEL
    try:
        parsed = extract_json(call_llm(description, selected_model))
        normalized = parsed.get("normalized_instruction") or description
    except Exception as exc:
        if debug:
            print(f"LLM 解析失败，回退本地规则: {exc}", file=sys.stderr)
        normalized = description
    try:
        return build_validated_task(normalized, description)
    except PlanningError as normalized_exc:
        if normalized != description:
            try:
                return build_validated_task(description, description)
            except PlanningError as original_exc:
                raise LLMPlanningError(f"3-DOF 规划失败: {normalized_exc}; 原始输入回退也失败: {original_exc}") from original_exc
        raise LLMPlanningError(f"3-DOF 规划失败: {normalized_exc}") from normalized_exc


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="调用 LLM 生成 3-DOF 机械臂 JSON 指令序列")
    parser.add_argument("description")
    parser.add_argument("--model", default=None)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--compact", action="store_true")
    parser.add_argument("--local-first", action="store_true", help="跳过 LLM，优先尝试本地规则解析规范输入")
    args = parser.parse_args()
    try:
        task = build_task_with_llm(args.description, args.model, args.debug, args.local_first)
    except LLMPlanningError as exc:
        print(f"LLM 规划失败: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(task, ensure_ascii=False, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
