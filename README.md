# 3-DOF 机械臂自然语言控制原型

自然语言任务经 LLM 归一化后，由本地 3-DOF 运动规划器生成 JSON 指令，并在终端或 Web 3D 界面演示。

## 机械臂模型

- `J1`：底座 yaw，绕 `Z` 轴旋转，决定目标方位。
- `J2`：肩关节 pitch。
- `J3`：肘关节 pitch。
- HOME：`{"j1": 0.0, "j2": 90.0, "j3": 0.0}`。
- 安全转向姿态：`{"j2": 55.0, "j3": -90.0}`。
- 几何：底座高 `260mm`，上臂 `280mm`，前臂 `240mm`，最大臂展 `520mm`。
- 限位：`J1=-180..180°`，`J2=-45..120°`，`J3=-150..150°`。
- TCP 必须满足 `z >= 0mm`。

规划路线会先展开 `J2/J3`，再旋转 `J1` 对准目标，之后由 `J2/J3` 协同伸出、下降、抬起和回收。

## 文件结构

- `arm_planner.py`：3-DOF 正/逆运动学、路径规划和任务校验。
- `llm_planner.py`：LLM 语义归一化，然后调用本地规划器。
- `random_task_generator.py`：随机生成自然语言任务。
- `task_demo.py`：终端轨迹演示。
- `workflow_demo.py`：随机任务、LLM 规划、终端演示串联。
- `web_ui.py`：HTTP API 与 Web 静态资源服务。
- `web/`：Three.js 3D 前端、交互和样式。
- `schemas/arm_task.schema.json`：任务 JSON schema。
- `prompts/system_prompt.md`：LLM 语义归一化提示词。
- `examples/grasp_box.json`：当前 3-DOF 抓取示例。
- `AGENTS.md`：项目上下文和维护注意事项。

## 安装与配置

```powershell
pip install -r requirements.txt
```

复制 `.env_example` 为 `.env`，填写：

```env
OPENAI_API_KEY=你的 API Key
OPENAI_MODEL=gpt-4.1-mini
# OPENAI_BASE_URL=https://你的网关地址/v1
```

如果 LLM 调用失败，`llm_planner.py` 会尽量回退到本地规则；Web 完整工作流仍建议配置 `.env`。

## 运行

Web 控制台：

```powershell
python .\web_ui.py
```

浏览器打开：

```text
http://127.0.0.1:8000
```

命令行：

```powershell
python .\arm_planner.py "抓取前面30厘米处地面上的盒子"
python .\llm_planner.py "抓取左侧30厘米处地面上的盒子"
python .\llm_planner.py "抓取左侧30厘米处地面上的盒子" | python .\task_demo.py
python .\workflow_demo.py -n 3
```

## JSON 约定

- 顶层包含 `task_id`、`task_description`、`created_at`、`metadata`、`steps`。
- `move_joints.params.joints` 必须包含 `j1/j2/j3`。
- `move_joints.params.tcp` 必须包含 `x/y/z`。
- Web 帧额外包含 `points.base/shoulder/elbow/tool`，用于 3D 渲染。
- 动作类型只有 `move_joints`、`gripper`、`wait`。

## 动画与速度

- 后端时间线：`MOVE_DURATION_MS = {"slow": 1200, "medium": 900, "fast": 650}`。
- 夹爪动作：`GRIPPER_DURATION_MS = 420`。
- 前端播放间隔：`FRAME_INTERVAL_MS = 60`。
- 如果修改这些值，建议同时检查 `web_ui.py`、`task_demo.py` 和 `web/app.js` 的时间线表现。

## 验证

```powershell
python -m py_compile .\arm_planner.py .\llm_planner.py .\random_task_generator.py .\task_demo.py .\workflow_demo.py .\web_ui.py
node --check .\web\app.js
python .\arm_planner.py "抓取左侧30厘米处地面上的盒子" --compact | python .\task_demo.py
```

Web 交互异常时优先检查：

```powershell
node --check .\web\app.js
```
