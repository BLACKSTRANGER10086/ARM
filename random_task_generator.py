"""随机中文机械臂任务生成模块。

负责生成抓取、放置、抓取后放置、复位类自然语言任务，用于测试和 Web 随机任务入口。
主要对外接口是 `generate_task()`。本模块被 `workflow_demo.py` 和
`web_ui.py` 调用，为后续 LLM 结构化语义解析和本地规划提供输入。
"""

import argparse
import random
import sys


OBJECTS = ["盒子", "杯子", "瓶子", "零件", "小物体", "方块", "工具"]
DIRECTIONS = ["前面", "后面", "左侧", "右侧"]
SURFACES = ["地面上", "桌面上", "台面上"]
DISTANCES_CM = [15, 20, 25, 30, 35, 40, 45, 50]


def random_location() -> str:
    return f"{random.choice(DIRECTIONS)}{random.choice(DISTANCES_CM)}厘米处{random.choice(SURFACES)}"


def random_grasp_task() -> str:
    return random.choice([
        "抓取{location}的{obj}",
        "把{location}的{obj}抓起来",
        "从{location}拾取{obj}",
    ]).format(location=random_location(), obj=random.choice(OBJECTS))


def random_place_task() -> str:
    return random.choice([
        "把物体放到{location}",
        "将手里的物体放置到{location}",
        "移动到{location}并放下物体",
    ]).format(location=random_location())


def random_pick_place_task() -> str:
    return random.choice([
        "把{source}的{obj}抓起来然后放到{destination}",
        "从{source}拾取{obj}，再放到{destination}",
        "先抓取{source}的{obj}，然后放置到{destination}",
    ]).format(source=random_location(), destination=random_location(), obj=random.choice(OBJECTS))


def random_home_task() -> str:
    return random.choice(["回到初始位置", "机械臂复位", "回到 home 位置"])


def generate_task(task_type: str = "mixed") -> str:
    if task_type == "grasp":
        return random_grasp_task()
    if task_type == "place":
        return random_place_task()
    if task_type in {"pick_place", "compound"}:
        return random_pick_place_task()
    if task_type == "home":
        return random_home_task()
    return random.choices([random_grasp_task, random_place_task, random_pick_place_task, random_home_task], weights=[0.48, 0.24, 0.2, 0.08], k=1)[0]()


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="随机生成 3-DOF 机械臂自然语言任务")
    parser.add_argument("-n", "--count", type=int, default=1)
    parser.add_argument("--type", choices=["mixed", "grasp", "place", "pick_place", "home"], default="mixed")
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()
    if args.seed is not None:
        random.seed(args.seed)
    for _ in range(max(args.count, 0)):
        print(generate_task(args.type))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
