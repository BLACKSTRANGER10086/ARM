# AGENTS.md：3-DOF 机械臂项目上下文

## 项目定位

这是一个 **3-DOF 机械臂自然语言控制原型**。当前链路为：

```text
自然语言任务
  -> LLM 输出结构化语义命令 JSON
  -> 本地 3-DOF 规划器编译为可执行任务 JSON
  -> validate_task() 校验
  -> 终端/Web 3D 演示
```

LLM 负责理解自然语言并输出高层语义命令，例如 `pick/place/home`；它不允许直接输出关节角、轨迹点或 `move_joints`。机械臂轨迹必须由本地确定性规划器生成并校验。

## 机械臂模型

- `J1`：底座 yaw，绕 `Z` 轴旋转，负责目标方位。
- `J2`：肩关节 pitch。
- `J3`：肘关节 pitch。
- HOME：`{"j1": 0.0, "j2": 90.0, "j3": 0.0}`。
- 安全转向姿态：`{"j2": 55.0, "j3": -90.0}`，旋转 `J1` 前先展开肩肘。
- 几何：底座高 `260mm`，上臂 `280mm`，前臂 `240mm`，最大臂展 `520mm`。
- 限位：`J1=-180..180°`，`J2=-45..120°`，`J3=-150..150°`。
- TCP 约束：末端高度必须满足 `z >= 0mm`。

## JSON 分层

项目中有两层 JSON，不能混淆：

### LLM 语义命令 JSON

由 `llm_planner.py` 请求模型生成，只描述任务意图：

```json
{
  "description": "先抓取右侧20厘米处地面上的杯子，然后放到前面5厘米处",
  "commands": [
    {
      "type": "pick",
      "target": {
        "direction": "right",
        "distance_mm": 200,
        "surface": "ground",
        "object": "杯子"
      }
    },
    {
      "type": "place",
      "target": {
        "direction": "front",
        "distance_mm": 50,
        "surface": "ground"
      }
    }
  ]
}
```

支持的语义命令：

- `pick`：从目标点抓取物体。
- `place`：把当前夹持物放到目标点。
- `home`：回到 HOME。

`target.direction` 支持 `front/back/left/right` 或中文方向；`distance_mm` 使用毫米；`surface` 支持 `ground/table/platform` 或中文表面。

### 本地可执行任务 JSON

由 `arm_planner.py` 生成，包含实际可演示动作：

- 顶层包含 `task_id`、`task_description`、`created_at`、`metadata`、`steps`。
- `steps[].action` 只允许 `move_joints`、`gripper`、`wait`。
- `move_joints.params.joints` 必须包含 `j1/j2/j3`。
- `move_joints.params.tcp` 必须包含 `x/y/z`。
- `metadata.semantic_plan` 会保留 LLM 输出的语义命令 JSON。

## 核心实现

- `arm_planner.py` 是运动规划核心：
  - `forward_kinematics_points(j1, j2, j3)` 返回 `base/shoulder/elbow/tool` 四个 3D 点。
  - `forward_kinematics(j1, j2, j3)` 返回 TCP `x/y/z`。
  - `inverse_kinematics(x, y, z)` 使用两连杆 IK 计算 `J2/J3`，并由 `x/y` 计算 `J1`。
  - `build_task(description)` 保留旧的本地中文规则解析，支持抓取、放置、复位、简单抓取后放置。
  - `build_task_from_command_plan(plan, original_description)` 将 LLM 语义 `commands` 编译为可执行任务 JSON。
  - `validate_task(task)` 校验动作字段、关节限位、插值过程、TCP 高度和基座对准阶段约束。
- `llm_planner.py` 默认优先调用 LLM 输出结构化语义命令 JSON；如果 LLM 不可用或语义命令不可规划，则回退本地规则。
- `--local-first` 或 Web 里的“本地优先（跳过 LLM）”只用于调试和无 API Key 场景，正常自然语言流程不建议勾选。
- 轨迹规划会在旋转底座前进入安全转向姿态；每个完整任务默认最后回 HOME。
- 动作时长由 `MOVE_DURATION_MS = {"slow": 1200, "medium": 900, "fast": 650}` 和 `GRIPPER_DURATION_MS = 420` 控制。

## Web 工作流

- `web_ui.py` 提供 HTTP 静态文件和 API：
  - `POST /api/random`：生成随机自然语言任务，类型支持 `mixed/grasp/place/pick_place/home`。
  - `POST /api/workflow`：接收 `task_text/model/retries/local_first`，返回任务 JSON 和 Web 动画帧。
- `/api/workflow` 要求 `task_text` 非空；空输入不会自动生成随机任务。
- Web 帧字段：
  - `joints`: `j1/j2/j3`
  - `position`: TCP `x/y/z`
  - `points`: `base/shoulder/elbow/tool`
  - `gripper`
  - `elapsed_ms`
- `web/app.js` 使用 Three.js 渲染三关节结构，并按 `FRAME_INTERVAL_MS = 60` 对后端关键帧插值播放。

## 文件职责

- `README.md`：项目说明、安装、运行、两层 JSON 约定和验证命令。
- `AGENTS.md`：项目上下文、关键约束和维护注意事项。
- `arm_planner.py`：3-DOF 正/逆运动学、语义命令编译、路径规划和任务校验。
- `llm_planner.py`：LLM 调用、结构化语义 JSON schema、本地规划回退。
- `random_task_generator.py`：随机中文任务生成，含抓取后放置任务。
- `task_demo.py`：终端轨迹演示，展示目标分析和执行时间线。
- `workflow_demo.py`：随机任务、LLM/本地规划、终端演示串联。
- `web_ui.py`：HTTP 服务、Web API、动画关键帧构建。
- `web/app.js`：按钮交互、HTTP 请求、Three.js 3D 动画和播放控制。
- `schemas/arm_task.schema.json`：本地可执行任务 JSON schema，不是 LLM 语义命令 schema。
- `prompts/system_prompt.md`：LLM 结构化语义命令提示词说明。
- `examples/grasp_box.json`：当前 3-DOF 抓取示例。

## 常用命令

```powershell
python .\web_ui.py
python .\arm_planner.py "抓取左侧30厘米处地面上的盒子"
python .\arm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处"
python .\llm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处"
python .\llm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" | python .\task_demo.py
python .\workflow_demo.py --type pick_place --local-first -n 1
```

验证：

```powershell
python -m py_compile .\arm_planner.py .\llm_planner.py .\random_task_generator.py .\task_demo.py .\workflow_demo.py .\web_ui.py
node --check .\web\app.js
```

## 维护注意事项

- 不要让 LLM 直接输出 `move_joints`、关节角或轨迹点；LLM 只能输出高层语义 `commands`。
- 修改语义命令 schema 时，需要同步更新 `llm_planner.py`、`arm_planner.py`、`prompts/system_prompt.md`、`README.md` 和相关示例。
- 修改关节数量、限位、连杆长度或帧字段时，必须同步更新 `arm_planner.py`、`web_ui.py`、`web/app.js`、`task_demo.py`、`schemas/arm_task.schema.json`、`README.md`、`prompts/system_prompt.md`、`examples/grasp_box.json`。
- Web 按钮无响应或 3D 不显示时，优先运行 `node --check .\web\app.js`。
- 浏览器显示旧效果时，先用 `Ctrl + F5` 强制刷新；`web_ui.py` 对静态资源设置了 `Cache-Control: no-store`。
