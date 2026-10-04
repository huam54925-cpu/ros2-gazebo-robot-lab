# Robot Skills：模型从安全 Frontier 中选择一个

当前链路：**模型选择候选 ID → MCP → 本地任务账本/仲裁 → 当前地图重新规划与路径检查 → Nav2 → 独立 guard → 里程计停车确认 → 本地任务结果 → 模型解释**。

本页描述单次 Frontier 与六工具基础接口。新增[有界连续循环和可选观察工具](bounded-exploration-agent.md)保留这些约束；`model-once` 仍只运行一次，`model-loop --observations` 才增加两个观察工具。没有任意坐标导航、模型速度控制或长期自主探索，机器人仍是本地 Docker 仿真。

## 六个 MCP 工具

| 工具 | 作用 |
|---|---|
| `get_robot_state()` | 读取位姿、地图摘要、雷达、导航/guard 观测与新鲜度 |
| `get_decision_context()` | 只读汇总上述状态、已有候选、最近/活动任务；不会刷新规划、移动或解除停车锁 |
| `get_safe_frontiers()` | 复用有效候选，或由本地算法生成候选并调用 Nav2 计算路径；不发导航目标 |
| `execute_frontier(frontier_id, request_id)` | 异步提交一个候选 ID，返回 task_id；执行前重新检查 |
| `get_task_status(task_id)` | 查询本地权威任务记录 |
| `stop_robot(request_id)` | 锁住后续运动，取消活动目标并暂停 Nav2；必须继续查询停车是否确认 |

所有工具都拒绝额外参数。`execute_frontier` 只接受两个字符串，不接受 `x/y/yaw/speed/clearance` 或 shell 文本。MCP 工具定义本身与模型侧 JSON schema 都限制参数；服务端不依赖模型遵守提示词。

候选使用带生成批次的 ID，例如 `F_abcdef123456_1`，不会把旧编号重新指向另一个位置。候选有效期为 120 秒。空集合表示没有当前合格候选，不能解释为探索完成。

## 任务语义

执行接口统一包含 `request_id / task_id / accepted / status / running / stopped / reason / sensor_fresh / map_version`。

- `request_id` 是调用者提供的标准 UUID；`task_id` 由本地账本分配。同一请求与相同参数重放原任务，并标注 `replayed_without_execution`；相同 ID 搭配不同参数会报冲突。
- `accepted` 仅表示进入本地执行队列，不代表 Nav2 已接收目标，更不是成功。
- `running` 表示本地执行器正在工作；`phase` 区分 validating、planning、navigating、stopping、finished。
- 终态为 succeeded、rejected、aborted、canceled、stop_unconfirmed 或 indeterminate。只有本地结果决定终态，模型解释不会修改它。
- `stopped` 来自停车验证。`sensor_fresh` 和 `sensor_observed_unix_s` 是任务采样时的传感器证据；历史任务完成后不会自动变成新的实时观测。实时状态应查询 `get_robot_state`。
- 地图版本包含栅格内容、尺寸、分辨率、原点与 frame；ROS 状态桥和规划执行器使用相同算法。

SQLite 账本位于 `workspace/log/robot-skills/tasks.sqlite3`。事务保证并发请求只接纳一个运动任务；高优先级停车不受该限制。每个任务由独立本地 supervisor 执行，MCP 客户端断开不删除任务。不要手工删除任务记录来绕过未确认的停车状态。

## 执行检查与停车

候选生成复用激进 Frontier 的位姿/footprint 过滤、信息收益代理和 Nav2 可达性规划，仅输出通过整条路径雷达间距检查的候选。执行时检查 ID、有效期、地图、位姿安全性和本地任务占用，再重新规划。地图在检查过程中变化则重查，连续变化不能确认时拒绝发目标。

执行中复用已有地图/路径变化重查、传感器与定位超时、guard 触发取消和停车确认。固定保护阈值为 **1.9 m**，规划要求 **2.15 m**。候选路径上限 8 m，目标运行上限 180 秒，累计里程超过 10 m 触发取消；这些是触发条件，不是制动距离硬保证。专用行为树没有自动倒车、旋转恢复或无限重试。

默认 baseline 模式中，成功或失败的已尝试位置都会进入后续候选排除集合。新增的 [memory 模式](regional-exploration.md)改为按 mission/epoch、位姿/朝向、有效收益与局部失败证据过滤；不把通行旧位置画成障碍。同一批候选 ID 不能用新 request_id 反复执行。新的地图也不会让模型绕过当前位置的重新规划和安全检查。

`stop_robot` 立即写入持久化停车锁，执行器响应取消；停车任务同时请求 Nav2 取消目标、暂停导航生命周期并使用新鲜里程计验证稳定低速。接口返回 accepted 时尚未确认停车。异常或任务超时可能触发 supervisor 停止 `robot-nav2`，独立 guard 继续在仿真容器内运行。

停车锁只能通过本地显式 `resume` 恢复，MCP 不提供解除锁工具。恢复前要求无活动/未解决任务且已停车；新停车请求与恢复竞争时，恢复不能覆盖更新的停车锁。

若监督进程已停止 Nav2 且留下 indeterminate / stop_unconfirmed，先用本地 `recover <task_id>` 确认 Nav2 未运行、guard 输出为零以及多次新鲜里程计低速，再标记该故障已解决。原失败状态与原因保留，停车锁仍不解除。之后重新启动 Nav2 并显式 resume；MCP 不提供 recover 或 resume。

与原 Frontier 和固定前进试验共用导航所有者锁。原 `robot-motion-trial.sh` 的新请求也转入这套任务框架；历史请求仍重放历史结果。外部 RViz 或任意 ROS 发布者不受应用层任务账本控制，实验时不得从其他客户端并发控制小车。本版未验收任意进程故障、动态障碍全场景或真实硬件。

## 使用

先启动仿真、SLAM、Nav2 和状态桥，见[只读链路运行说明](robot-readonly.md)。已有服务运行时：

```bash
cd /mnt/robot_disk/ros_sim
./scripts/robot-skills.sh state
./scripts/robot-skills.sh frontiers
./scripts/robot-skills.sh context

# 模型从本地安全集合选择一次，自动等待本地终态，然后解释结果
./scripts/robot-skills.sh model-once --output logs/skills-model-frontier.json

# 使用同一候选生成器、执行器和限制，按经典激进 Frontier 分数选一次
./scripts/robot-skills.sh classical-once --output logs/skills-classical-frontier.json
```

手动按 ID 提交、查询及停车：

```bash
./scripts/robot-skills.sh execute <frontier_id> --request-id <UUID>
./scripts/robot-skills.sh status <task_id>
./scripts/robot-skills.sh stop --request-id <新的UUID>
./scripts/robot-skills.sh status <停车任务task_id>

# 确认停车和任务收尾完成后，显式本地恢复；不是模型工具
./scripts/robot-skills.sh resume
```

异常停止 Nav2 后的本地恢复流程：

```bash
./scripts/robot-skills.sh recover <未确认任务的task_id>
./docker/start-nav2.sh  # 保持此终端运行
# 另一个终端等待 Nav2 就绪后
./scripts/robot-skills.sh resume
```

`model-once` 最多一次选择、一次解释，不自动另选目标重试。候选、决策上下文、模型选择、任务 ID、最终执行结果均保存在输出 JSON。规划与执行细节在 `workspace/log/robot-skills/`。API 密钥留在本地模型客户端，ROS 执行器不读取密钥。

## 实验解释

模型与经典策略入口共用候选生成、评分信息、Nav2、SLAM、路径检查和停止逻辑。但连续在同一已变化的地图上各跑一次不构成公平对照；比较决策质量必须重置相同地图、机器人起点、初始扫描和预算，重复运行并记录真实新增已知面积、行驶里程、耗时、失败及最小雷达距离。

本次启动时地图没有合格候选，先如实返回空集合，再使用已有本地初始化扫描建立探索起点。完整转向路径检查通过，扫描最近雷达距离 3.064 m，结束后确认停车；这一步没有暴露为模型 observe 工具。

OpenAI 调用流程依据[官方 Function calling 文档](https://developers.openai.com/api/docs/guides/function-calling)。候选信息收益是代理量；不得把它当作执行后的真实收益或全屋覆盖率。

## 2026-10-04 仿真验收记录

- 官方接口模型 `gpt-6.1-sol` 从六个本地安全候选中选择 `F_f4738d4257eb_2`。请求 `edef56a8-ad6d-4efc-b45b-56bf3a239932`，任务 `f460bdeb-7950-443d-9b3b-555bbcd3a9f7`；重新规划约 2.008 m，整条路径预测最小雷达间距 2.991 m。
- 本地 Nav2 终态为 succeeded / goal_reached，停车确认 true；里程计累计 **1.832 m**，最近雷达 **3.519 m**，已知地图面积由 136.880 增至 140.370 m²（增量 **3.490 m²**）。最终里程计线速度、角速度均为零。抵达遵守 Nav2 的位置/朝向容差，不要求走满路径长度。
- 同 request_id 经真实 MCP 重放返回原任务，任务数不变；附加 x 参数被 MCP 拒绝。没有再发运动任务。
- 运动期间停车测试：固定前进任务在 0.046 m 后 canceled 且 stopped=true；stop_robot 任务 succeeded，新运动请求被 stop_latched 拒绝；MCP 客户端断开不影响监督执行器完成停车。后经本地显式 resume 恢复。
- 第一次 Frontier 试运行发现 SQLite 连接未及时关闭导致描述符耗尽。监督进程停止 Nav2，并保留 indeterminate / stopped=false，不宣称成功。已修复连接关闭，加入 1,200 次读操作的文件描述符回归测试；显式停车恢复记录保留原失败状态，后续候选排除了尝试过的位置。本次成功执行中抽查执行器仅 35 个打开的文件描述符。
- 自动验证共 **57 项通过**：任务账本 11、API/MCP/原接口 20、路径间距 12、探索安全 10、取消失败注入 4。取消超时且生命周期暂停失败时不能仅凭静止里程计宣称已安全取消，必须进入未确认状态并由监督进程处理。

证据：`logs/skills-model-frontier-validated.json`、`logs/skills-replay-validation.json`、`logs/skills-stop-validation.json`、`logs/skills-failure-recovery.json`。首次故障记录 `logs/skills-model-frontier.json` 保留。当前只证明这一仿真闭环可运行；尚未执行多次相同初始条件的经典/模型决策质量对照。
