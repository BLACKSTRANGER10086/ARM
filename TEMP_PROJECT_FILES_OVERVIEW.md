# 3-DOF 机械臂项目文件功能与实现逻辑（临时）

> 临时说明文档，用于快速理解当前项目结构。核心原则：LLM 只做自然语言语义归一化，关节轨迹必须由本地确定性规划器生成并通过校验。

## 1. 整体链路

项目运行链路为：

```text
自然语言任务
  -> llm_planner.py 语义归一化（失败则回退原始输入）
  -> arm_planner.py 本地规划与校验
  -> JSON steps
  -> task_demo.py 终端演示 / web_ui.py + web/app.js Web 3D 演示
```

规划输出的动作类型只有三种：

- `move_joints`：移动 `j1/j2/j3` 三个关节，并记录 TCP。
- `gripper`：控制夹爪打开或闭合。
- `wait`：等待指定毫秒数。

三关节定义：

- `J1`：底座 yaw，绕 Z 轴转向目标方位。
- `J2`：肩关节 pitch。
- `J3`：肘关节 pitch。

关键安全策略是：旋转 `J1` 前先让 `J2/J3` 到安全转向姿态，底座对准阶段只允许 `J1` 改变，之后下降、夹取、抬起阶段保持已对准方位。

## 2. 根目录文件

### `arm_planner.py`

项目最核心的确定性运动规划器，负责 3-DOF 正/逆运动学、路径生成、任务 JSON 构建和安全校验。

主要实现逻辑：

- 用常量定义机械臂几何、HOME 姿态、关节限位、安全转向姿态、TCP 最小高度、动作时长等约束。
- `Point3D` 和 `ArmPoints` 用 dataclass 表示空间点和四个关键机械臂节点：`base/shoulder/elbow/tool`。
- `planar_forward()` 先在二维径向平面计算肩肘和末端位置，再由 `forward_kinematics_points()` 根据 `J1` yaw 旋转到三维坐标。
- `forward_kinematics()` 返回 TCP，也就是 `tool` 点。
- `inverse_kinematics()` 先由 `x/y` 算 `J1`，再用两连杆 IK 算 `J2/J3`；同时枚举肘部两个解，过滤超限或 TCP 低于地面的姿态，最后选更接近安全转向姿态的解。
- `parse_target()` 从中文任务中提取距离、方向和高度：前/后/左/右映射到 `x/y`，地面、桌面、台面映射到不同 `z`。
- `plan_grasp()` 生成抓取流程：预开夹爪、肩肘展开、安全转向、伸到目标上方、下降抓取、闭合、等待、抬起、回安全姿态、回 HOME。
- `plan_place()` 生成放置流程：安全转向、伸出、下降、打开夹爪、等待、回收、回 HOME。
- `plan_home()` 只生成回 HOME 的动作。
- `build_task()` 根据任务描述分派到抓取、放置或复位规划，并生成 `task_id/created_at/metadata/steps`。
- `validate_task()` 是最后防线：校验字段、步骤编号、动作类型、关节限位、TCP 高度、插值过程安全，以及底座对准阶段只能改变 `J1`。
- 命令行入口会把任务 JSON 输出到 stdout，规划失败时向 stderr 输出错误。

维护重点：

- 修改关节数量、连杆长度、限位、动作类型时，必须同步修改 schema、终端演示、Web 帧生成和前端渲染。
- 不要绕过 `validate_task()` 输出轨迹。

### `llm_planner.py`

LLM 语义归一化层。它不生成轨迹，只把用户自然语言改写成更清晰、可规划的中文任务，再交给 `arm_planner.py`。

主要实现逻辑：

- `load_dotenv()` 从 `.env` 加载 `OPENAI_API_KEY/OPENAI_MODEL/OPENAI_BASE_URL`。
- `create_client()` 创建 OpenAI 客户端，缺少依赖或 API Key 会抛出 `LLMPlanningError`。
- `schema()` 定义模型输出结构，只允许返回 `normalized_instruction`。
- `call_llm()` 调用 Responses API，让模型输出结构化 JSON；提示词强调机械臂只有 yaw、肩、肘三自由度。
- `extract_json()` 从模型返回文本中截取 JSON。
- `build_task_with_llm()` 先尝试 LLM 归一化；失败时使用原始描述回退；随后调用 `build_task()` 生成本地轨迹，并再次 `validate_task()`。
- 命令行入口输出完整任务 JSON，可传 `--model/--debug/--compact`。

维护重点：

- LLM 返回内容只能作为“任务描述”，不能作为关节轨迹使用。
- 如果调整支持任务类型，需要同步更新这里的提示和本地规划器。

### `random_task_generator.py`

随机中文任务生成器，用于测试和 Web 随机按钮。

主要实现逻辑：

- 内置对象、方向、表面和距离候选池。
- `random_location()` 组合方向、距离和表面。
- `random_grasp_task()` 生成抓取类任务。
- `random_place_task()` 生成放置类任务。
- `random_home_task()` 生成复位任务。
- `generate_task()` 根据 `mixed/grasp/place/home` 类型返回任务；`mixed` 按权重更偏向抓取。
- 命令行支持 `-n/--count`、`--type`、`--seed`。

### `task_demo.py`

终端轨迹演示器，读取任务 JSON 并以文本方式模拟每一步。

主要实现逻辑：

- `load_task()` 从文件或 stdin 读取 JSON；如果输入中混有日志，会尝试截取最外层 `{...}`。
- `simulate_task()` 先调用 `validate_task()`，然后从 HOME 开始逐步执行：
  - `move_joints`：更新关节，调用正运动学计算 TCP，累计移动时长。
  - `gripper`：更新夹爪状态，累计夹爪时长。
  - `wait`：累计等待时长。
- 输出内容包括任务信息、几何参数、目标分析、每步关节变化、TCP、夹爪状态和总时长。
- 命令行默认从 stdin 读取，因此可以接收 `llm_planner.py` 的管道输出。

### `workflow_demo.py`

命令行端到端演示脚本，把随机任务、LLM 规划和终端演示串起来。

主要实现逻辑：

- 根据 `--count/--type/--seed` 生成一批自然语言任务。
- 对每个任务调用 `build_task_with_llm()`，失败时按 `--max-retries` 重试。
- 可选通过 `--save-json-dir` 保存任务 JSON。
- 成功后打印原始任务、规划 JSON 和 `task_demo.simulate_task()` 的终端演示。

### `web_ui.py`

Web 后端，提供静态页面和两个 API，并把任务 JSON 转成前端动画关键帧。

主要实现逻辑：

- `make_frame()` 根据当前关节和夹爪状态调用正运动学，生成单帧：
  - `joints`
  - `position`
  - `points.base/shoulder/elbow/tool`
  - `gripper`
  - `elapsed_ms`
- `build_frames()` 从 HOME 初始帧开始遍历任务 steps，按动作类型更新关节、夹爪和时间线。
- `plan_with_retries()` 封装 LLM 规划重试。
- `run_workflow()` 接收前端 payload；如果没有任务文本则生成随机任务；然后返回 `task_text/task/frames`。
- `WebHandler`：
  - `GET /` 返回 `web/index.html`
  - `GET /app.js` 返回前端 JS
  - `GET /styles.css` 返回样式
  - `POST /api/random` 返回随机任务
  - `POST /api/workflow` 返回完整规划结果和动画帧
- `main()` 启动 `ThreadingHTTPServer`，默认地址为 `http://127.0.0.1:8000`。

维护重点：

- Web 帧字段变化时要同步 `web/app.js`。
- 这里生成的是关键帧；细分动画插值由前端完成。

### `README.md`

项目面向用户的说明文档，覆盖项目定位、机械臂模型、安装配置、运行方式、JSON 约定、动画时长和验证命令。

维护重点：

- 用户可见行为、命令、JSON 字段、模型参数变化时应同步更新。

### `AGENTS.md`

面向代码代理或维护者的项目上下文文件，记录本项目的关键约束、文件职责、常用命令和维护注意事项。

维护重点：

- 当项目架构或安全约束变化时，这个文件要优先保持准确。

### `.env_example`

环境变量模板，说明需要配置的 OpenAI API Key、默认模型和可选 Base URL。

主要用途：

- 复制为 `.env` 后供 `llm_planner.py` 读取。
- 不应包含真实密钥。

### `.env`

本地私有配置文件，供 `llm_planner.py` 加载 API Key、模型和 Base URL。

维护重点：

- 不要把真实密钥写入文档或提交到公共仓库。

### `requirements.txt`

Python 依赖声明，目前只要求 `openai>=1.0.0`。

### `TEMP_PROJECT_FILES_OVERVIEW.md`

当前临时文档，也就是本文。用途是快速说明各文件功能和实现逻辑，后续可删除或整理并入 `README.md`。

## 3. `web/` 前端文件

### `web/index.html`

Web 控制台页面结构。

主要实现逻辑：

- 页面分成四块：自然语言输入、3D 演示、时间线、JSON 指令。
- 提供任务文本框、随机类型选择、重试次数、模型名输入、随机生成和运行按钮。
- 提供播放、暂停、重置和帧滑块。
- 通过 import map 从 CDN 引入 Three.js 和 OrbitControls。
- 以 ES module 方式加载 `/app.js`。

### `web/app.js`

前端交互和 Three.js 3D 动画核心。

主要实现逻辑：

- 缓存所有 DOM 元素，定义 HOME、机械臂几何和 `FRAME_INTERVAL_MS = 60`。
- 初始化 Three.js 场景、相机、渲染器、灯光、网格、坐标轴和 OrbitControls。
- `post()` 封装 API 请求。
- `payload()` 从页面表单收集任务文本、类型、重试次数和模型名。
- `frameFromApi()` 规范化后端帧；如果旧数据缺少 `j3` 或 `points`，则使用前端 `forward()` 兜底计算。
- `forward()` 在前端复刻 3-DOF 正运动学，用于插值过程和旧帧兼容。
- `buildMotionFrames()` 把后端关键帧按照 `elapsed_ms` 和 60ms 间隔细分成播放帧，并对 yaw 使用最短角度插值。
- `renderFrame()` 每帧重建机械臂模型：底座、连杆、关节球、夹爪和轨迹线。
- `addLink()` 用圆柱体连接两个 3D 点，`addGripper()` 根据夹爪宽度和状态渲染末端执行器。
- `updateReadout()` 更新帧滑块、当前步骤、关节角和夹爪状态。
- `runWorkflow()` 调 `/api/workflow`，得到 JSON 和帧后构建动画并自动播放。
- 底部绑定按钮、滑块和 resize 事件，并持续执行 `animate()` 渲染循环。

维护重点：

- 如果修改后按钮无响应，优先运行 `node --check .\web\app.js`。
- 若后端帧格式变化，需要同步 `frameFromApi()`、`buildMotionFrames()` 和 `renderFrame()`。

### `web/styles.css`

Web 控制台样式。

主要实现逻辑：

- 定义黑、白、红色强调色和暗色界面变量。
- 用 CSS Grid 构建输入区、3D 舞台、时间线和 JSON 区。
- 设置按钮、表单、状态、遥测信息、步骤列表和代码块样式。
- `@media (max-width: 1100px)` 下切换为单列布局，适配较窄屏幕。

## 4. `schemas/` 文件

### `schemas/arm_task.schema.json`

任务 JSON 的结构约束。

主要实现逻辑：

- 顶层要求 `task_id/task_description/created_at/steps`。
- `steps` 中每个 step 要求 `step/action/params`。
- `action` 只允许 `move_joints/gripper/wait`。
- `move_joints` 要求：
  - `joints.j1/j2/j3`，并限制对应角度范围。
  - `tcp.x/y/z`，其中 `z >= 0`。
  - `speed` 只能是 `slow/medium/fast`。
- `gripper` 要求 `state/width/force`，并限制夹爪开口和力范围。
- `wait` 要求非负 `duration_ms`。

维护重点：

- 它是外部 JSON 约定的静态描述；实际安全校验仍以 `arm_planner.validate_task()` 为准。

## 5. `prompts/` 文件

### `prompts/system_prompt.md`

LLM 语义归一化提示词文档，说明模型只能输出规范中文任务描述，不能输出关节轨迹。

主要实现逻辑：

- 约束 LLM 角色为自然语言解析器。
- 说明机械臂三自由度能力边界。
- 要求保留任务类型、方向、距离、表面高度和目标物。
- 对超出 3-DOF 能力的姿态要求，只保留可规划的位置意图。

注意：

- 当前 `llm_planner.py` 使用的是代码内联 prompt，不直接读取这个 Markdown；如果希望提示词单一来源，可以后续改成读取此文件。

## 6. `examples/` 文件

### `examples/grasp_box.json`

抓取左侧 30 厘米处地面盒子的示例任务 JSON。

主要内容：

- 顶层 metadata 记录模型、几何、限位和目标分析。
- steps 展示完整抓取路径：预开夹爪、安全展开、旋转 J1 对准左侧、伸到目标上方、下降抓取、闭合、等待、抬起、回安全姿态、旋回 HOME、收拢回 HOME。
- 可用作 schema、终端演示和 Web 动画的回归样例。

## 7. 生成目录和本地目录

### `__pycache__/`

Python 运行时生成的字节码缓存目录，不属于源码逻辑。

### `.idea/`

IDE 配置目录，通常只影响本地开发体验。

### `.skills/`

本地工具或代理技能目录，不属于机械臂业务运行链路。

## 8. 建议验证命令

修改 Python 或前端逻辑后建议运行：

```powershell
python -m py_compile .\arm_planner.py .\llm_planner.py .\random_task_generator.py .\task_demo.py .\workflow_demo.py .\web_ui.py
node --check .\web\app.js
```

端到端快速检查：

```powershell
python .\arm_planner.py "抓取左侧30厘米处地面上的盒子" --compact | python .\task_demo.py
python .\workflow_demo.py -n 1 --type grasp --seed 1
```
