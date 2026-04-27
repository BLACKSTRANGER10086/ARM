# 3-DOF 机械臂自然语言控制台

用自然语言控制三自由度机械臂的原型系统。输入一句话，系统自动完成语义理解、运动规划、安全校验和 3D 可视化演示。

```
"把右侧20厘米处的杯子抓起来放到前面"
```

## 目录

**使用指南**
- [工作原理](#工作原理)
- [快速开始](#快速开始)
- [使用方式](#使用方式) — [Web 界面](#web-界面) · [命令行](#命令行)

**系统设计**
- [两层 JSON 设计](#两层-json-设计) — [LLM 语义命令](#第一层llm-语义命令) · [本地可执行任务](#第二层本地可执行任务)
- [机械臂模型](#机械臂模型) — [轨迹规划策略](#轨迹规划策略) · [动画与速度](#动画与速度)
- [Web API](#web-api)

**开发参考**
- [架构详解](#架构详解) — [模块职责](#模块职责) · [核心函数](#核心函数) · [LLM 回退机制](#llm-回退机制) · [状态管理](#状态管理) · [错误处理](#错误处理)
- [目录结构](#目录结构)
- [验证](#验证)
- [维护注意事项](#维护注意事项)

---

## 工作原理

系统采用两层架构，将语义理解与运动规划严格分离：

```
自然语言 → LLM 语义解析 → 本地运动规划 → 安全校验 → 3D 演示
              ↓                ↓
        语义命令 JSON       可执行任务 JSON
     (pick/place/home)   (move_joints/gripper/wait)
```

**核心约束：** LLM 只输出高层语义命令（抓取/放置/复位），绝不能输出关节角、轨迹点或 `move_joints`。所有关节角计算、逆运动学、轨迹生成和安全校验均由本地 `arm_planner.py` 完成。LLM 不可用时会尝试回退到本地中文规则解析。

## 快速开始

### 1. 安装依赖

```powershell
pip install -r requirements.txt
```

需要 Python 3.10+（使用了 `X | Y` 类型联合语法）。唯一的 Python 外部依赖是 `openai>=1.0.0`，Web 前端另从 CDN 加载 Three.js。

### 2. 配置环境变量

复制 `.env_example` 为 `.env` 并填写：

```env
OPENAI_API_KEY=sk-你的api-key
OPENAI_MODEL=deepseek-v4-flash
OPENAI_BASE_URL=https://你的api地址/v1
```

- `OPENAI_API_KEY`：必填，支持任何 OpenAI 兼容接口。
- `OPENAI_MODEL`：可选，默认 `deepseek-v4-flash`。
- `OPENAI_BASE_URL`：使用第三方 API 时必填，填写 OpenAI 兼容接口根路径，例如 `https://你的api地址/v1`，不要填到 `/chat/completions` 这一级。
- `ARM_LOCAL_FIRST`：可选，设为 `1` 可全局优先使用本地规则解析，等同于 `--local-first`。

> 不填 API Key 也能运行——LLM 调用会失败并尝试回退到本地规则解析，功能受限但足以测试运动规划。

> 兼容性提示：部分国产模型或第三方 API（如 DeepSeek、GLM、聚合中转服务）可能不支持 OpenAI 的 Responses API `/responses`，只支持 Chat Completions `/chat/completions`。项目会先尝试 `/responses`，失败后自动降级到 `/chat/completions`；如果 Web 状态栏显示“LLM 调用失败，已使用本地规则”，请优先检查 `OPENAI_BASE_URL` 是否是正确的 `/v1` 根路径，以及该服务是否已发布兼容接口。

### 3. 启动

**Web 界面**（推荐）：

```powershell
python web_ui.py
# 浏览器打开 http://127.0.0.1:8000
```

**命令行**：

```powershell
python arm_planner.py "抓取前面30厘米处地面上的盒子"
```

## 使用方式

### Web 界面

启动 `python web_ui.py` 后在浏览器中操作：

1. 在输入框输入自然语言任务（如"抓取左侧30厘米处地面上的盒子"）
2. 点击 **[ 运行流程 ]**，系统完成 LLM 解析 → 本地规划 → 3D 动画播放
3. 也可以点 **[ 随机生成 ]** 自动填入一条随机任务

页面分为四个区域：

- **指令输入**：任务文本、随机类型选择、重试次数、本地优先开关、模型名
- **3D 机械臂视图**：Three.js 渲染的三关节结构和运动轨迹
- **执行时间线**：每一步动作的概要
- **任务 JSON**：本地生成的完整可执行任务 JSON

**按钮说明**：

- **[ 随机生成 ]**：调用 `/api/random` 生成随机自然语言任务填入输入框，不执行规划。
- **[ 运行流程 ]**：调用 `/api/workflow` 执行完整的 LLM 解析 → 本地规划 → 动画播放。
- **本地优先**：勾选后优先尝试本地中文规则解析，本地失败或不匹配时仍会调用 LLM。适合调试或无 API Key 场景，正常自然语言测试建议不勾选。
- **播放/暂停/重置**：只控制前端已拿到的动画帧，不重新规划。

**连续运动**：Web 端默认不回 HOME，服务端记忆当前关节和夹爪状态，每次命令从上一次结束位置继续。输入"回零"或"复位"并点击运行可回到初始位置，也可调用 `POST /api/reset`。

**缓存问题**：浏览器显示旧效果时，使用 `Ctrl + F5` 强制刷新。`web_ui.py` 对静态资源已设置 `Cache-Control: no-store`。

### 命令行

**本地规划器**（不调用 LLM，输入需为规范中文格式）：

```powershell
python arm_planner.py "抓取前面30厘米处地面上的盒子"
python arm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" --no-return-home
```

调用链路：

```
规范中文任务 → arm_planner.build_task() → 本地规则解析目标
  → 正/逆运动学和路径规划 → validate_task() → 输出可执行任务 JSON
```

支持规范中文的抓取、放置、复位和简单"抓取后放置"，不适合复杂自然语言。

**LLM 规划器**（支持自由自然语言）：

```powershell
python llm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处"
```

调用链路：

```
自然语言任务 → llm_planner.call_llm() → LLM 输出 description + commands
  → arm_planner.build_task_from_command_plan() → validate_task()
  → 输出可执行任务 JSON
```

加 `--local-first` 时会先尝试 `arm_planner.build_task()` 本地规则，失败后再调 LLM。

**终端轨迹演示**（管道）：

```powershell
python llm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" | python task_demo.py
```

`task_demo.py` 从 stdin 读取已生成的任务 JSON，校验后打印任务信息、每步关节变化、TCP、夹爪状态和总耗时。它不重新规划。

**随机端到端工作流**：

```powershell
# 单次
python workflow_demo.py --type pick_place -n 1

# 多轮连续运动（不回 HOME，每轮从上次位置继续）
python workflow_demo.py --type pick_place --no-return-home -n 3

# 优先本地规则（失败时仍调 LLM）
python workflow_demo.py --type pick_place --local-first -n 1
```

调用链路：

```
random_task_generator.py 生成自然语言任务
  → llm_planner.py 生成语义 commands 并本地规划
  → task_demo.py 输出终端演示
```

`--no-return-home` 时 `workflow_demo.py` 通过 `extract_end_state()` 提取每轮结束状态，下一轮从当前位置继续。

**常用选项**：

| 选项 | 适用脚本 | 说明 |
|------|----------|------|
| `--no-return-home` | `arm_planner` / `llm_planner` / `workflow_demo` | 执行后不回 HOME，停在目标方向的安全转向姿态 |
| `--local-first` | `llm_planner` / `workflow_demo` | 优先尝试本地中文规则解析，失败或不匹配时仍会调用 LLM |
| `--model NAME` | `llm_planner` / `workflow_demo` | 指定 LLM 模型名 |
| `--compact` | `arm_planner` / `llm_planner` | 输出紧凑 JSON（无缩进） |
| `--debug` | `llm_planner` / `workflow_demo` | 显示 LLM 回退和解析失败信息 |
| `--max-retries N` | `workflow_demo` | LLM 规划失败时最大重试次数，默认 3 |
| `--seed N` | `workflow_demo` | 随机种子，便于复现 |
| `--save-json-dir DIR` | `workflow_demo` | 将生成的任务 JSON 保存到指定目录 |

---

> **系统设计** — 两层 JSON、机械臂模型、轨迹策略、API

## 两层 JSON 设计

这是本项目的核心架构决策——将 LLM 的语义理解能力与确定性运动规划解耦。

### 第一层：LLM 语义命令

由 `llm_planner.py` 请求模型生成，只描述任务意图，不涉及执行细节：

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
- `home`：回到 HOME 位置。

`target` 字段说明：

- `direction`：`front/back/left/right` 或中文方向。
- `distance_mm`：距离，毫米。也支持 `distance_cm`（厘米）。
- `surface`：`ground/table/platform` 或中文表面。
- `object`：目标物体名称。

### 第二层：本地可执行任务

由 `arm_planner.py` 将语义命令编译为关节级执行步骤：

```json
{
  "task_id": "uuid",
  "task_description": "抓取左侧30厘米处地面上的盒子",
  "created_at": "2026-04-25T01:14:03+08:00",
  "metadata": {
    "model": "3-DOF yaw+shoulder+elbow",
    "base_height_mm": 260,
    "upper_arm_mm": 280,
    "forearm_mm": 240,
    "arm_reach_mm": 520,
    "home_joints": { "j1": 0.0, "j2": 90.0, "j3": 0.0 },
    "joint_limits_deg": { "j1": [-180, 180], "j2": [-45, 120], "j3": [-150, 150] },
    "semantic_plan": {
      "description": "抓取左侧30厘米处地面上的盒子",
      "commands": [{ "type": "pick", "target": { "direction": "left", "distance_mm": 300, "surface": "ground", "object": "盒子" } }]
    }
  },
  "steps": [
    { "step": 1, "action": "gripper",     "comment": "预开夹爪",    "params": { "state": "open", "width": 80, "force": 0 } },
    { "step": 2, "action": "move_joints", "comment": "肩肘展开到安全转向姿态", "params": { "joints": { "j1": 0, "j2": 55, "j3": -90 }, "tcp": { "x": 357, "y": 0, "z": 351 }, "speed": "medium" } },
    { "step": 3, "action": "move_joints", "comment": "旋转基座对准目标方位",  "params": { "joints": { "j1": 90, "j2": 55, "j3": -90 }, "tcp": { "x": 0, "y": 357, "z": 351 }, "speed": "medium" } },
    { "step": 4, "action": "move_joints", "comment": "肩肘协同伸出到目标上方", "params": { "joints": { "j1": 90, "j2": 40.86, "j3": -109.29 }, "tcp": { "x": 0, "y": 300, "z": 220 }, "speed": "medium" } },
    { "step": 5, "action": "move_joints", "comment": "保持方位下降到抓取点",  "params": { "joints": { "j1": 90, "j2": 20.29, "j3": -101.33 }, "tcp": { "x": 0, "y": 300, "z": 120 }, "speed": "slow" } },
    { "step": 6, "action": "gripper",     "comment": "闭合夹爪抓取", "params": { "state": "close", "width": 20, "force": 50 } },
    { "step": 7, "action": "wait",        "comment": "等待夹持稳定", "params": { "duration_ms": 500 } }
  ]
}
```

`action` 只有三种：`move_joints`（关节运动）、`gripper`（夹爪）、`wait`（等待）。`move_joints` 必须同时包含 `joints`（关节角）和 `tcp`（末端坐标）。`metadata.semantic_plan` 保存 LLM 输出的语义命令对象（仅通过 LLM 路径生成的任务包含此字段，本地规则直接生成的任务不含）。完整示例见 `examples/grasp_box.json`，JSON Schema 见 `schemas/arm_task.schema.json`。

## 机械臂模型

```
J1（底座 yaw）—— 绕 Z 轴旋转，决定目标方位
J2（肩关节 pitch）—— 上臂俯仰
J3（肘关节 pitch）—— 前臂俯仰

几何参数：底座高 260mm / 上臂 280mm / 前臂 240mm / 最大臂展 520mm
关节限位：J1 ±180° / J2 -45°~120° / J3 ±150°
安全约束：TCP 高度 z ≥ 0mm

HOME 位：{ j1: 0°, j2: 90°, j3: 0° }（竖直收拢，TCP 位于正上方 780mm）
安全转向位：{ j2: 55°, j3: -90° }（旋转基座前先展开肩肘，避免碰撞）
```

### 轨迹规划策略

典型的抓取轨迹经过以下阶段：

1. **预开夹爪** — 确保夹爪打开
2. **进入安全转向姿态** — J2/J3 展开到 `{55°, -90°}`
3. **旋转基座对准** — 保持 J2/J3 不变，J1 旋转到目标方位
4. **伸出到目标上方** — J2/J3 协同调整，到达悬停点
5. **下降到抓取点** — 慢速接近目标
6. **闭合夹爪** — 夹取物体
7. **抬起** — 慢速上升
8. **回收** — 回到安全转向姿态或 HOME

不回 HOME 时停在目标方向的安全转向姿态，下次命令从该位置继续。

### 动画与速度

- 后端关节运动时长：`slow=1200ms`、`medium=900ms`、`fast=650ms`
- 夹爪动作时长：`420ms`
- 前端播放间隔：`60ms`（对后端关键帧进行插值）

---

> **开发参考** — 架构、状态管理、验证、维护规则

## 架构详解

### 模块职责

| 模块 | 职责 |
|------|------|
| `arm_planner.py` | 运动学引擎（唯一的关节轨迹生成与安全检查层）：正/逆运动学、语义命令编译（`build_task_from_command_plan()`）、本地中文规则解析（`build_task()`）、路径规划、`validate_task()` 校验、`extract_end_state()` 状态提取 |
| `llm_planner.py` | LLM 结构化语义 JSON 生成（`call_llm()`）；通过 `build_task_with_llm()` 串联 LLM 调用与本地规划；失败时回退本地规则 |
| `random_task_generator.py` | 随机生成中文自然语言任务（抓取、放置、抓取后放置、复位），用于测试 |
| `task_demo.py` | 终端轨迹可视化：从 stdin 或文件读取任务 JSON，校验后打印关节变化、TCP 和耗时。主要接口 `simulate_task()` |
| `workflow_demo.py` | 端到端命令行演示：串联随机任务生成 → LLM/本地规划 → 终端演示，支持多轮链式运动 |
| `web_ui.py` | HTTP 服务器：静态文件服务、API 端点、动画关键帧生成（`build_frames()`）；维护线程安全的机械臂状态 |
| `web/app.js` | Three.js 3D 渲染器：前端按 60ms 间隔对后端关键帧插值播放，跟踪当前帧状态 |
| `web/index.html` | 控制台页面结构 |
| `web/styles.css` | 页面样式 |
| `schemas/arm_task.schema.json` | 本地可执行任务的 JSON Schema（不是 LLM 语义命令的 schema） |
| `prompts/system_prompt.md` | LLM 结构化语义命令提示词 |
| `examples/grasp_box.json` | 3-DOF 抓取任务的完整示例 |

### 核心函数

| 函数 | 说明 |
|------|------|
| `forward_kinematics(j1, j2, j3)` | 正运动学，返回 TCP 坐标 `{x, y, z}` |
| `forward_kinematics_points(j1, j2, j3)` | 正运动学，返回 `base/shoulder/elbow/tool` 四个 3D 点 |
| `inverse_kinematics(x, y, z)` | 逆运动学，两连杆 IK 算 J2/J3，由 x/y 算 J1 |
| `build_task(description)` | 本地中文规则解析入口 |
| `build_task_from_command_plan(plan, ...)` | 将 LLM 语义 `commands` 编译为可执行任务 |
| `validate_task(task)` | 校验动作字段、关节限位、插值过程、TCP 高度和基座对准阶段约束 |
| `extract_end_state(task)` | 从已完成任务中提取最终关节角和夹爪状态，用于链式调用 |

规划函数（`build_task`、`build_task_from_command_plan`、`plan_grasp`、`plan_place` 等）均支持 `start_joints` 指定起始关节角和 `return_home` 控制是否回 HOME。`validate_task()` 支持 `start_joints` 但不涉及 `return_home`。`extract_end_state()` 不是规划函数，它仅从已完成任务中回放步骤以提取最终状态。

### LLM 回退机制

`llm_planner.build_task_with_llm()` 的完整决策流程：

```
1. 如果 local_first=true 且输入匹配本地规则 → 尝试 arm_planner.build_task()
   ├─ 成功 → 返回
   └─ 失败 → 继续

2. 调用 LLM 生成语义 JSON
   ├─ 先尝试 Responses API (/responses)
   ├─ 不支持 /responses 时自动降级 Chat Completions (/chat/completions)
   ├─ 返回含 commands → arm_planner.build_task_from_command_plan() → 返回
   └─ LLM 调用失败或 JSON 不可规划 → 继续

3. 回退：用 LLM 返回的 normalized_instruction 或原始输入
   → arm_planner.build_task() 本地规则解析 → 返回或抛出错误
```

国产模型或第三方 OpenAI 兼容服务的接口能力不完全一致。常见情况是 `/v1/chat/completions` 可用，但 `/v1/responses` 返回 404（例如“API does not exist or has not been published”）。这不是模型不可用，而是接口路径不支持；当前实现会自动切到 Chat Completions。Web 端会在状态栏和任务 `metadata.planner_source` 中标记实际来源：`llm` 表示 LLM 语义解析成功，`fallback_local` 表示 LLM 调用失败后回退本地规则，`local_first` 表示本地优先生效。

### 状态管理

**Web 端**：`web_ui.py` 使用 `threading.Lock` 保护 `_arm_state`（关节角 + 夹爪状态），每次 `/api/workflow` 从当前状态起始规划，成功后更新状态，失败时状态不变。`/api/reset` 可重置为 HOME。`/api/random` 在调用方传入 `seed` 时创建请求级 `random.Random(seed)` 实例，避免线程间 PRNG 污染；不传 `seed` 时使用模块全局随机实例。

**CLI 端**：`arm_planner.py` 和 `llm_planner.py` 默认从 HOME 起始，默认回 HOME。`--no-return-home` 关闭回 HOME。`workflow_demo.py` 多轮模式下通过 `extract_end_state()` 在内存中链接各轮状态。

### 错误处理

目标超出机械臂可达范围（520mm）或违反关节限位时，`arm_planner.py` 抛出 `PlanningError`：

- Web 端：异常被 `web_ui.py` 捕获，返回 HTTP 500 + 错误信息，前端显示错误，机械臂状态不变。
- CLI 端：异常向上传播，打印错误信息后退出。

## Web API

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/workflow` | POST | 从当前状态规划任务，返回任务 JSON 和动画帧，成功后更新状态 |
| `/api/random` | POST | 生成随机自然语言任务文本 |
| `/api/reset` | POST | 将机械臂关节和夹爪状态重置为 HOME |

### `/api/workflow`

请求体：

```json
{
  "task_text": "抓取前面30厘米处地面上的盒子",
  "model": null,
  "retries": 3,
  "local_first": false,
  "return_home": false
}
```

- `task_text`：必填，不能为空。
- `return_home`：默认 `false`，设为 `true` 可在任务结束后回 HOME。

响应体：

```json
{
  "task_text": "...",
  "task": { "task_id": "...", "steps": [...] },
  "frames": [
    {
      "step": 0,
      "action": "init",
      "joints": { "j1": 0, "j2": 90, "j3": 0 },
      "position": { "x": 0, "y": 0, "z": 780 },
      "points": {
        "base": { "x": 0, "y": 0, "z": 0 },
        "shoulder": { "x": 0, "y": 0, "z": 260 },
        "elbow": { "x": 0, "y": 0, "z": 540 },
        "tool": { "x": 0, "y": 0, "z": 780 }
      },
      "gripper": { "state": "open", "width": 100, "force": 0 },
      "elapsed_ms": 0
    }
  ]
}
```

### `/api/random`

请求体：

```json
{ "type": "mixed" }
```

`type` 可选 `mixed/grasp/place/pick_place/home`。支持可选 `seed` 字段用于复现。

响应体：

```json
{ "task_text": "抓取前面30厘米处地面上的盒子" }
```

### `/api/reset`

无请求体。响应：

```json
{ "status": "ok", "joints": { "j1": 0, "j2": 90, "j3": 0 } }
```

## 目录结构

```
├── arm_planner.py             # 运动学引擎：正/逆运动学、轨迹生成、安全校验
├── llm_planner.py             # LLM 语义解析，失败时回退本地规则
├── random_task_generator.py   # 随机中文任务生成
├── task_demo.py               # 终端轨迹可视化
├── workflow_demo.py           # 端到端命令行工作流
├── web_ui.py                  # Web 服务器、API、动画帧构建
├── web/
│   ├── index.html             # 控制台页面
│   ├── app.js                 # Three.js 3D 渲染与交互
│   └── styles.css             # 样式
├── schemas/
│   └── arm_task.schema.json   # 可执行任务 JSON Schema
├── prompts/
│   └── system_prompt.md       # LLM 提示词
├── examples/
│   └── grasp_box.json         # 抓取任务示例
├── .env_example               # 环境变量模板
└── requirements.txt           # Python 依赖
```

## 验证

项目没有测试框架，使用以下命令验证：

```powershell
# 语法检查
python -m py_compile arm_planner.py llm_planner.py random_task_generator.py task_demo.py workflow_demo.py web_ui.py
node --check web/app.js

# 冒烟测试：本地规划 + 终端演示
python arm_planner.py "把右侧20厘米处地面上的杯子抓起来然后放到前面5cm处" --compact | python task_demo.py
```

Web 交互异常时优先检查前端语法：

```powershell
node --check web/app.js
```

## 维护注意事项

### 跨文件修改规则

修改**语义命令 schema**（如新增命令类型或 target 字段）时需同步更新：

- `llm_planner.py`（schema 定义和 prompt）
- `arm_planner.py`（命令编译逻辑）
- `prompts/system_prompt.md`
- `README.md`
- 相关示例文件

修改**关节数量、限位、连杆长度或帧字段**时需同步更新：

- `arm_planner.py`
- `web_ui.py`
- `web/app.js`
- `task_demo.py`
- `schemas/arm_task.schema.json`
- `README.md`
- `prompts/system_prompt.md`
- `examples/grasp_box.json`

### 状态相关

`web_ui.py` 维护线程安全的机械臂状态（`_arm_state`）。修改状态字段时需同步更新 `extract_end_state()`、`build_frames()` 和 `/api/reset` 的初始值。
