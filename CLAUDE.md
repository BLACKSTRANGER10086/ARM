# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指引。

## 项目概述

3-DOF 机械臂自然语言控制原型。处理链路：
自然语言 → LLM 输出语义命令 JSON（`pick/place/home`）→ 本地规划器编译为可执行任务 JSON（关节轨迹）→ 终端或 Web 3D 演示。

**核心约束：** LLM 只能输出高层语义 `commands`，不能输出 `move_joints`、关节角或轨迹点。所有运动学计算、路径规划和安全校验均由 `arm_planner.py` 在本地完成。

## 常用命令

```powershell
# 安装依赖
pip install -r requirements.txt

# 启动 Web 界面 (http://127.0.0.1:8000)
python web_ui.py

# 测试本地规划器（不调用 LLM）
python arm_planner.py "抓取前面30厘米处地面上的盒子"

# 不回 HOME，停在安全转向姿态
python arm_planner.py "抓取前面30厘米处地面上的盒子" --no-return-home

# 测试 LLM 规划器
python llm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处"

# 终端演示管道
python llm_planner.py "..." | python task_demo.py

# 随机端到端工作流
python workflow_demo.py --type pick_place -n 1
python workflow_demo.py --type pick_place --local-first -n 1

# 多轮连续运动（不回 HOME，每轮从上次位置继续）
python workflow_demo.py --type pick_place --no-return-home -n 3

# 语法验证（项目无测试框架，使用以下命令）
python -m py_compile arm_planner.py llm_planner.py random_task_generator.py task_demo.py workflow_demo.py web_ui.py
node --check web/app.js

# 冒烟测试：本地规划 + 终端演示
python arm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" --compact | python task_demo.py
```

## 架构

两层 JSON 设计将 LLM 职责与运动规划分离：

1. **LLM 语义命令**（`llm_planner.py`）：`{description, commands: [{type: "pick"|"place"|"home", target: {direction, distance_mm, surface, object}}]}`
2. **本地可执行任务 JSON**（`arm_planner.py`）：`{task_id, steps: [{action: "move_joints"|"gripper"|"wait", params: {joints: {j1,j2,j3}, tcp: {x,y,z}}}]}`

### 核心模块

- **`arm_planner.py`** — 运动学引擎：正/逆运动学、语义命令编译、轨迹生成、`validate_task()` 校验。支持 `start_joints` 指定起始位置和 `return_home` 控制是否回 HOME。`extract_end_state()` 提取任务结束状态用于链式调用。也包含旧版本地中文规则解析（`build_task()`）。
- **`llm_planner.py`** — 调用 LLM 生成结构化语义 JSON；LLM 失败时回退到本地规则。`--local-first` 优先尝试本地规则解析（失败后仍调 LLM），`--no-return-home` 不回 HOME。
- **`web_ui.py`** — HTTP 服务器，维护线程安全的机械臂状态（关节角+夹爪），提供 `POST /api/random`、`POST /api/workflow` 和 `POST /api/reset`。每次 workflow 从当前状态起始，默认不回 HOME。
- **`web/app.js`** — Three.js 3D 渲染器；以 60ms 间隔对后端关键帧插值播放。跟踪当前帧状态，动画间保持上次结束位置。
- **`task_demo.py`** — 终端轨迹可视化（从 stdin 读取 JSON）。
- **`workflow_demo.py`** — 端到端命令行：随机任务生成 → 规划 → 终端演示。
- **`random_task_generator.py`** — 随机生成中文自然语言任务。

### 机械臂模型

- 关节：J1（底座 yaw）、J2（肩关节 pitch）、J3（肘关节 pitch）
- HOME 位：`{j1: 0, j2: 90, j3: 0}`；安全转向姿态：`{j2: 55, j3: -90}`
- 几何参数：底座高 260mm，上臂 280mm，前臂 240mm，最大臂展 520mm
- 限位：J1 ±180°、J2 -45°..120°、J3 ±150°；TCP z >= 0mm

## 环境配置

复制 `.env_example` 为 `.env`，填写 `OPENAI_API_KEY`、`OPENAI_MODEL`（默认 `deepseek-v4-flash`）和可选的 `OPENAI_BASE_URL`。使用 OpenAI 兼容 API。

## 状态与复位

Web 端服务器维护机械臂当前状态（关节角+夹爪），每次 `/api/workflow` 从当前位置起始规划，默认不回 HOME。CLI 默认回 HOME，可通过 `--no-return-home` 关闭。用户输入"回零"或调用 `POST /api/reset` 可回到 HOME。

## 跨文件修改规则

修改**语义命令 schema** 时需同步更新：`llm_planner.py`、`arm_planner.py`、`prompts/system_prompt.md`、`README.md` 及相关示例。

修改**关节数量、限位、连杆长度或帧字段**时需同步更新：`arm_planner.py`、`web_ui.py`、`web/app.js`、`task_demo.py`、`schemas/arm_task.schema.json`、`README.md`、`prompts/system_prompt.md`、`examples/grasp_box.json`。
