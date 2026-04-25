# 3-DOF 机械臂项目文件功能与实现逻辑（临时）

> 临时说明文档，用于快速理解当前项目结构。当前核心原则：LLM 输出高层语义 `commands` JSON；关节轨迹仍由本地确定性规划器生成并通过 `validate_task()` 校验。

## 1. 整体链路

```text
自然语言任务
  -> llm_planner.py 调用 LLM 输出结构化语义命令 JSON
  -> arm_planner.py 将 commands 编译为 move_joints/gripper/wait
  -> validate_task() 校验轨迹
  -> task_demo.py 终端演示 / web_ui.py + web/app.js Web 3D 演示
```

LLM 输出的是语义层：

```json
{
  "description": "先抓取右侧20厘米处地面上的杯子，然后放到前面5厘米处",
  "commands": [
    {"type": "pick", "target": {"direction": "right", "distance_mm": 200, "surface": "ground", "object": "杯子"}},
    {"type": "place", "target": {"direction": "front", "distance_mm": 50, "surface": "ground"}}
  ]
}
```

本地规划器输出的是执行层：

- `move_joints`：移动 `j1/j2/j3` 三个关节，并记录 TCP。
- `gripper`：控制夹爪打开或闭合。
- `wait`：等待指定毫秒数。

## 2. 根目录文件

### `arm_planner.py`

项目核心规划器。它同时支持两类入口：

- `build_task(description)`：旧式本地中文规则入口，支持抓取、放置、复位和简单抓取后放置。
- `build_task_from_command_plan(plan, original_description)`：当前 LLM 主流程入口，把语义 `commands` 编译为可执行任务 JSON。

关键职责：

- 定义机械臂几何、HOME、关节限位、安全转向姿态、TCP 最小高度和动作时长。
- 提供正运动学 `forward_kinematics_points()` / `forward_kinematics()`。
- 提供两连杆逆运动学 `inverse_kinematics()`。
- 将 `pick/place/home` 命令序列编译为安全轨迹。
- 通过 `validate_task()` 校验字段、步骤编号、动作类型、关节限位、TCP 高度、插值过程和底座对准阶段约束。

### `llm_planner.py`

自然语言语义层。默认调用 LLM，让模型输出结构化语义命令 JSON。

关键职责：

- 从 `.env` 加载 `OPENAI_API_KEY/OPENAI_MODEL/OPENAI_BASE_URL`。
- 用 Responses API 的 JSON schema 约束模型输出 `description` 和 `commands`。
- 调用 `arm_planner.build_task_from_command_plan()` 生成最终任务 JSON。
- 在 LLM 调用失败或语义命令不可规划时回退到 `arm_planner.build_task()`。
- 支持 `--local-first` 或环境变量 `ARM_LOCAL_FIRST=1` 跳过 LLM，用本地规则调试。

### `random_task_generator.py`

随机中文任务生成器。支持 `mixed/grasp/place/pick_place/home`，其中 `pick_place` 会生成“抓起来然后放到...”类复合任务。

### `task_demo.py`

终端演示器。读取任务 JSON 后先调用 `validate_task()`，再按步骤打印关节变化、TCP、夹爪状态、耗时，以及单目标、复合目标或语义命令目标分析。

### `workflow_demo.py`

命令行端到端演示脚本。串联随机任务生成、LLM/本地规划和 `task_demo.simulate_task()`；支持 `--type pick_place` 和 `--local-first`。

### `web_ui.py`

Web 后端。负责静态资源、随机任务 API、工作流 API 和动画关键帧构建。

- `POST /api/random`：生成随机自然语言任务。
- `POST /api/workflow`：要求 `task_text` 非空，接收 `model/retries/local_first`，返回 `task_text/task/frames`。
- `build_frames()` 把本地可执行任务 JSON 转成 Web 关键帧。

### `README.md`

用户向说明文档，描述运行方式、两层 JSON、Web API 和验证命令。

### `AGENTS.md`

维护者上下文文档，记录架构边界、关键约束、文件职责和维护注意事项。

### `.env_example` / `.env`

OpenAI 兼容接口配置。`.env` 可设置 `OPENAI_API_KEY`、`OPENAI_MODEL`、`OPENAI_BASE_URL` 和 `ARM_LOCAL_FIRST`。

### `requirements.txt`

Python 依赖，目前主要是 `openai>=1.0.0`。

## 3. `web/` 前端文件

### `web/index.html`

Web 控制台页面结构。提供自然语言输入、随机类型选择、重试次数、模型名、本地优先开关、运行按钮、播放控制、时间线和任务 JSON 显示区。

### `web/app.js`

前端交互和 Three.js 动画核心。

关键职责：

- 从表单组装 `/api/workflow` payload。
- 每次运行前清空旧结果，避免显示上一条随机任务。
- 将后端关键帧按 `FRAME_INTERVAL_MS = 60` 插值为播放帧。
- 使用 Three.js 渲染底座、肩肘连杆、夹爪和 TCP 轨迹。

### `web/styles.css`

Web 控制台样式，包含暗色布局、表单、按钮、遥测、时间线、JSON 面板和本地优先复选框样式。

## 4. `schemas/` 文件

### `schemas/arm_task.schema.json`

描述本地可执行任务 JSON，也就是 `steps` 层，不描述 LLM 语义 `commands` 层。

## 5. `prompts/` 文件

### `prompts/system_prompt.md`

LLM 结构化语义命令提示词说明，要求模型只输出 `description` 和 `commands`，不能输出关节角或轨迹点。

注意：当前 `llm_planner.py` 使用代码内联 prompt；这个 Markdown 是提示词设计文档，二者变更时要同步。

## 6. `examples/` 文件

### `examples/grasp_box.json`

抓取左侧 30 厘米处地面盒子的可执行任务 JSON 示例，可用于 schema、终端演示和 Web 动画回归。

## 7. 生成目录和本地目录

- `__pycache__/`：Python 字节码缓存。
- `.idea/`：IDE 配置。
- `.skills/`：本地工具/代理技能目录，不属于机械臂业务链路。

## 8. 建议验证命令

```powershell
python -m py_compile .\arm_planner.py .\llm_planner.py .\random_task_generator.py .\task_demo.py .\workflow_demo.py .\web_ui.py
node --check .\web\app.js
python .\arm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" --compact | python .\task_demo.py
python .\workflow_demo.py -n 1 --type pick_place --local-first
```
