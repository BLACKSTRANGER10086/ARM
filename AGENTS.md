# AGENTS.md：3-DOF 机械臂项目上下文

## 项目定位

这是一个 **3-DOF 机械臂自然语言控制原型**。项目链路为：

```text
自然语言任务 -> LLM 语义归一化 -> 本地 3-DOF 运动规划 -> JSON 指令 -> 终端/Web 3D 演示
```

LLM 只负责把用户输入整理成明确、可规划的中文任务描述；机械臂轨迹必须由本地确定性规划器生成并校验。

## 机械臂模型

- `J1`：底座 yaw，绕 `Z` 轴旋转，负责目标方位。
- `J2`：肩关节 pitch。
- `J3`：肘关节 pitch。
- HOME：`{"j1": 0.0, "j2": 90.0, "j3": 0.0}`。
- 安全转向姿态：`{"j2": 55.0, "j3": -90.0}`，转动 `J1` 前先展开肩肘。
- 几何：底座高 `260mm`，上臂 `280mm`，前臂 `240mm`，最大臂展 `520mm`。
- 限位：`J1=-180..180°`，`J2=-45..120°`，`J3=-150..150°`。
- TCP 约束：末端高度必须满足 `z >= 0mm`。

## 核心实现

- `arm_planner.py` 是运动规划核心：
  - `forward_kinematics_points(j1, j2, j3)` 返回 `base/shoulder/elbow/tool` 四个 3D 点。
  - `forward_kinematics(j1, j2, j3)` 返回 TCP `x/y/z`。
  - `inverse_kinematics(x, y, z)` 使用两连杆 IK 计算 `J2/J3`，并由 `x/y` 计算 `J1`。
  - `validate_task()` 校验动作字段、关节限位、插值过程、TCP 高度和基座对准阶段约束。
- 任务动作类型只有：`move_joints`、`gripper`、`wait`。
- 路径规划顺序：预开夹爪 -> 肩肘展开 -> 底座对准 -> 肩肘伸出/下降 -> 夹爪动作 -> 抬起 -> 安全姿态回收 -> HOME。
- 动作时长由 `MOVE_DURATION_MS = {"slow": 1200, "medium": 900, "fast": 650}` 和 `GRIPPER_DURATION_MS = 420` 控制。

## Web 工作流

- `web_ui.py` 提供 HTTP 静态文件和 API：
  - `POST /api/random`：生成随机自然语言任务。
  - `POST /api/workflow`：生成任务 JSON 和 Web 动画帧。
- Web 帧字段：
  - `joints`: `j1/j2/j3`
  - `position`: TCP `x/y/z`
  - `points`: `base/shoulder/elbow/tool`
  - `gripper`
  - `elapsed_ms`
- `web/app.js` 使用 Three.js 渲染真实三关节结构，并对缺失 `j3` 或缺失 `points` 的旧数据做兜底。
- 前端播放节奏由 `FRAME_INTERVAL_MS = 60` 控制。

## 文件职责

- `README.md`：项目说明、安装、运行和 JSON 约定。
- `AGENTS.md`：项目上下文、关键约束和维护注意事项。
- `arm_planner.py`：3-DOF 正/逆运动学、路径规划和任务校验。
- `llm_planner.py`：LLM 调用、语义归一化、本地规划回退。
- `random_task_generator.py`：随机中文任务生成。
- `task_demo.py`：终端轨迹演示。
- `workflow_demo.py`：随机任务、LLM 规划、终端演示串联。
- `web_ui.py`：HTTP 服务、Web API、帧构建。
- `web/app.js`：按钮交互、HTTP 请求、Three.js 3D 动画。
- `schemas/arm_task.schema.json`：任务 JSON schema。
- `prompts/system_prompt.md`：LLM 语义归一化提示词。
- `examples/grasp_box.json`：当前 3-DOF 抓取示例。

## 常用命令

```powershell
python .\web_ui.py
python .\arm_planner.py "抓取左侧30厘米处地面上的盒子"
python .\llm_planner.py "抓取左侧30厘米处地面上的盒子"
python .\llm_planner.py "抓取左侧30厘米处地面上的盒子" | python .\task_demo.py
python .\workflow_demo.py -n 3
```

验证：

```powershell
python -m py_compile .\arm_planner.py .\llm_planner.py .\random_task_generator.py .\task_demo.py .\workflow_demo.py .\web_ui.py
node --check .\web\app.js
```

## 维护注意事项

- 修改关节数量、限位、连杆长度或帧字段时，必须同步更新 `arm_planner.py`、`web_ui.py`、`web/app.js`、`task_demo.py`、`schemas/arm_task.schema.json`、`README.md`、`prompts/system_prompt.md`、`examples/grasp_box.json`。
- 不要让 LLM 直接输出关节轨迹；轨迹必须由本地规划器生成并通过 `validate_task()`。
- Web 按钮无响应或 3D 不显示时，优先运行 `node --check .\web\app.js`，前端模块语法错误会阻断事件绑定和渲染初始化。
- 浏览器显示旧效果时，先用 `Ctrl + F5` 强制刷新缓存。
