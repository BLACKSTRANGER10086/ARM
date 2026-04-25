"""命令行端到端工作流演示模块。

负责串联随机任务生成、LLM 结构化语义解析、本地规划和终端演示。它调用
`random_task_generator.generate_task()`、`llm_planner.build_task_with_llm()`
和 `task_demo.simulate_task()`，用于快速验证自然语言到轨迹演示的完整链路。
"""

import argparse
import json
import random
import sys
from pathlib import Path

from llm_planner import LLMPlanningError, build_task_with_llm
from random_task_generator import generate_task
from task_demo import simulate_task


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="串联随机指令生成、LLM 语义 commands、本地规划和终端演示")
    parser.add_argument("-n", "--count", type=int, default=1, help="执行次数，默认 1")
    parser.add_argument(
        "--type",
        choices=["mixed", "grasp", "place", "pick_place", "home"],
        default="mixed",
        help="任务类型，默认 mixed",
    )
    parser.add_argument("--seed", type=int, default=None, help="随机种子，便于复现")
    parser.add_argument("--model", default=None, help="传给 llm_planner 的模型名")
    parser.add_argument("--debug", action="store_true", help="输出 LLM 解析失败和回退信息")
    parser.add_argument("--local-first", action="store_true", help="跳过 LLM，优先尝试本地中文规则解析")
    parser.add_argument("--max-retries", type=int, default=3, help="LLM 规划失败时的最大重试次数")
    parser.add_argument(
        "--save-json-dir",
        default=None,
        help="可选：将本地生成的可执行任务 JSON 保存到指定目录",
    )
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    save_dir = Path(args.save_json_dir) if args.save_json_dir else None
    if save_dir:
        save_dir.mkdir(parents=True, exist_ok=True)

    for index in range(max(args.count, 0)):
        task_text = generate_task(args.type)
        print(f"=== Run {index + 1} ===")
        print(f"Natural Language Task: {task_text}")

        task_json = None
        last_error = None
        for attempt in range(1, max(args.max_retries, 1) + 1):
            try:
                task_json = build_task_with_llm(task_text, args.model, args.debug, args.local_first)
                break
            except LLMPlanningError as exc:
                last_error = exc
                print(f"LLM planning retry {attempt}/{args.max_retries} failed: {exc}", file=sys.stderr)

        if task_json is None:
            print(f"LLM planning failed: {last_error}", file=sys.stderr)
            return 1

        if save_dir:
            output_path = save_dir / f"task_{index + 1}.json"
            output_path.write_text(json.dumps(task_json, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"Saved JSON: {output_path}")

        print("Planned JSON:")
        print(json.dumps(task_json, ensure_ascii=False, indent=2))
        print()
        print("Demo:")
        print(simulate_task(task_json))
        if index + 1 < args.count:
            print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
