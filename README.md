# 3-DOF 机械臂自然语言控制原型

这是一个用自然语言控制 3-DOF 机械臂的原型系统。当前实现采用分层架构：

```text
用户自然语言
  -> LLM 输出结构化语义命令 JSON
  -> 本地 3-DOF 规划器生成可执行任务 JSON
  -> 本地校验
  -> 终端或 Web 3D 演示
```

LLM 只负责理解任务并输出 `pick/place/home` 这类高层命令；关节角、TCP、轨迹步骤和安全校验都由本地代码完成。

## 机械臂模型

- `J1`：底座 yaw，绕 `Z` 轴旋转，决定目标方位。
- `J2`：肩关节 pitch。
- `J3`：肘关节 pitch。
- HOME：`{"j1": 0.0, "j2": 90.0, "j3": 0.0}`。
- 安全转向姿态：`{"j2": 55.0, "j3": -90.0}`。
- 几何：底座高 `260mm`，上臂 `280mm`，前臂 `240mm`，最大臂展 `520mm`。
- 限位：`J1=-180..180°`，`J2=-45..120°`，`J3=-150..150°`。
- TCP 必须满足 `z >= 0mm`。

规划路线会先展开 `J2/J3`，再旋转 `J1` 对准目标，之后由 `J2/J3` 协同伸出、下降、抬起和回收。完整任务默认最后回到 HOME。

## 两层 JSON

### 1. LLM 语义命令 JSON

`llm_planner.py` 请求模型输出结构化语义命令：

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

- `pick`：抓取目标。
- `place`：放置当前夹持物。
- `home`：回 HOME。

LLM 不应输出关节角、轨迹点或 `move_joints`。

### 2. 本地可执行任务 JSON

`arm_planner.py` 将语义命令编译为可执行任务：

- 顶层包含 `task_id`、`task_description`、`created_at`、`metadata`、`steps`。
- `steps[].action` 只有 `move_joints`、`gripper`、`wait`。
- `move_joints.params.joints` 包含 `j1/j2/j3`。
- `move_joints.params.tcp` 包含 `x/y/z`。
- `metadata.semantic_plan` 保存 LLM 输出的语义命令。

## 文件结构

- `arm_planner.py`：3-DOF 正/逆运动学、语义命令编译、路径规划和任务校验。
- `llm_planner.py`：LLM 结构化语义命令生成、本地规划回退。
- `random_task_generator.py`：随机生成抓取、放置、抓取后放置、复位任务。
- `task_demo.py`：终端轨迹演示。
- `workflow_demo.py`：随机任务、LLM/本地规划、终端演示串联。
- `web_ui.py`：HTTP API 与 Web 静态资源服务。
- `web/`：Three.js 3D 前端、交互和样式。
- `schemas/arm_task.schema.json`：本地可执行任务 JSON schema。
- `prompts/system_prompt.md`：LLM 结构化语义命令提示词。
- `examples/grasp_box.json`：3-DOF 抓取示例。
- `AGENTS.md`：项目上下文和维护注意事项。

## 安装与配置

```powershell
pip install -r requirements.txt
```

复制 `.env_example` 为 `.env`，填写：

```env
OPENAI_API_KEY=你的 API Key
OPENAI_MODEL=deepseek-v4-flash
# OPENAI_BASE_URL=https://你的网关地址/v1
```

如果 LLM 调用失败，`llm_planner.py` 会回退到本地规则解析。Web 和命令行都提供 `local_first` / `--local-first` 调试入口，用于跳过 LLM 直接测试本地规则。

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
python .\arm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处"
python .\llm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处"
python .\llm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" | python .\task_demo.py
python .\workflow_demo.py --type pick_place --local-first -n 1
```

说明：

- `llm_planner.py` 默认优先调用 LLM 输出语义 `commands`。
- `--local-first` 会跳过 LLM，直接用本地规则解析规范输入。
- Web 的“本地优先（跳过 LLM）”复选框与 `--local-first` 等价。

## Web API

- `POST /api/random`
  - 输入：`{"type": "mixed|grasp|place|pick_place|home"}`
  - 输出：`{"task_text": "..."}`
- `POST /api/workflow`
  - 输入：`{"task_text": "...", "model": null, "retries": 3, "local_first": false}`
  - 输出：`{"task_text": "...", "task": {...}, "frames": [...]}`

Web 帧字段：

- `joints`: `j1/j2/j3`
- `position`: TCP `x/y/z`
- `points`: `base/shoulder/elbow/tool`
- `gripper`
- `elapsed_ms`

## 动画与速度

- 后端时间线：`MOVE_DURATION_MS = {"slow": 1200, "medium": 900, "fast": 650}`。
- 夹爪动作：`GRIPPER_DURATION_MS = 420`。
- 前端播放间隔：`FRAME_INTERVAL_MS = 60`。

## 验证

```powershell
python -m py_compile .\arm_planner.py .\llm_planner.py .\random_task_generator.py .\task_demo.py .\workflow_demo.py .\web_ui.py
node --check .\web\app.js
python .\arm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" --compact | python .\task_demo.py
```

Web 交互异常时优先检查：

```powershell
node --check .\web\app.js
```
