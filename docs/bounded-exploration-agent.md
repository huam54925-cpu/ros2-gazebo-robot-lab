# 有界连续 Frontier 与主动观察

2026-10-04 实现。阶段 1 扩大决策轮数，阶段 2 用显式开关增加观察选项。模型只选本地候选 ID，不持有速度、坐标、路径、guard、恢复锁或 shell 权限。仍是本地仿真，不是长期自主探索或真实硬件验收。

后续已增加[区域探索与逐步地图收益](regional-exploration.md)。下文保留首版验收事实；新执行器对 MOVE/OBSERVE 统一保存地图窗口，非整数原点可计算诊断性交叠面积，但不能用于饱和判定。

## 阶段 1：连续 Frontier

```bash
cd /mnt/robot_disk/ros_sim
./scripts/robot-skills.sh model-loop --max-steps 3 \
  --max-distance 6 --max-sim-time 120 --max-wall-time 180 --max-failures 2 \
  --output logs/model-loop.json
```

默认 MCP 仍为原来的六个工具。每轮：读取上下文 → 生成/刷新安全候选 → 重新读取上下文 → 模型选择 ID → 本地重新规划与完整路径检查 → 等待终态及停车 → 读取最新状态 → 下一轮。每轮创建新的 UUID，原 request_id 的重试只查询原任务。

`max_steps` 限制提交次数（包括拒绝/失败），可设 1–5，默认 3。默认累计里程 6 m、仿真时间 120 s、墙钟 180 s、失败 2 次；任一条件先触发就结束。显式验收允许墙钟最多 900 s、仿真最多 180 s、里程最多 20 m、失败最多 3 次。慢仿真中 180 秒墙钟可能只够完成一次动作，不能保证完成三次。

另有 `no_safe_frontiers`、`no_safe_actions`、`candidate_generation_unavailable`、`no_action_fits_remaining_budget`、`operator_stop`、`model_stop`、`feedback_unavailable`、`simulation_clock_reset`、`stop_unconfirmed` 与客户端错误等终止原因。无候选不代表探索完成。触发预算后需要取消、暂停 Nav2 并验证停车，因此最终报告耗时可能超过预算；时间/里程上限是发出停止的触发点，不是制动距离或零延迟保证。

预算保存在本地 SQLite 会话记录，MCP 不接受预算参数。本地执行器检查累计里程与仿真/墙钟时间，独立 watchdog 同时检查预算与客户端心跳（15 秒超时）。其他客户端不能向活动会话插入运动任务；停车随时有优先权。模型响应返回时会再次检查会话状态，预算已经耗尽就不再派发目标。

每个循环结束都写入停车锁。MCP 没有 resume/recover。检查结果后，本地操作者可显式执行：

```bash
./scripts/robot-skills.sh resume
```

有未完成会话或停车未确认时拒绝恢复。不会在轮与轮之间自动解除保护锁。

## 阶段 2：MOVE / OBSERVE / STOP

```bash
./scripts/robot-skills.sh model-loop --observations --max-steps 3 \
  --max-wall-time 180 --output logs/model-loop-observations.json

# 只生成选项，不发运动目标
./scripts/robot-skills.sh observation-options

# 手动执行本地选项 ID；返回 task_id 后查询本地终态
./scripts/robot-skills.sh observe <option_id> --request-id <UUID>
./scripts/robot-skills.sh status <task_id>
```

`--observations` 的 MCP 增加两个工具，共八个：

- `get_observation_options()`：生成安全观察选项，不移动。
- `perform_observation(option_id, request_id)`：只接受两个 ID，异步执行并复用统一任务账本、仲裁、去重、取消、停车锁。

固定候选为原地左右 45°、90°，以及距当前位姿 0.75/1.25 m 的附近观察点（经规划后路径最多 2 m）。不提供任意角度、任意坐标或完整绕圈接口。旋转和附近换位都消耗 `max_steps`，没有观察次数无限、导航次数有限的漏洞。

旋转派发前读取实际雷达 TF、当前地图和 Nav2 配置中的 footprint/padding，检查所有中间角度的雷达轨迹与车体扫掠。occupied、unknown、地图外均不能进入车体扫掠。采样考虑雷达偏置与最远车体顶点半径，并留出点间余量。执行中按实际累计转角检查剩余有符号轨迹；地图变化、定位跳变、扫描低于阈值、超时都会取消。Nav2 Spin 的碰撞检查保持开启，stop_robot 同时取消导航和 Spin 并暂停 Nav2。

保护阈值保持 **1.9 m**，规划要求保持 **2.15 m**。模型不能以观察名义绕过 guard。附近换位复用现有 Nav2 与整条路径复核。

## 收益与重复限制

当前是 360°雷达。收益模型采用地图固定方向射线，从真实偏置雷达位置计算沿途未知可见格并集，扣除当前和近期最多八个视点的集合。零偏置且位置不变时，转向本身的几何面积收益为零。已知障碍阻断射线，预测最长 8 m、未知深度最多 2 m；这是乐观面积代理，不是测得的熵下降。观察选项至少有 0.25 m² 新颖性代理才列入。

字段包括 `novel_unknown_area_proxy_m2`、`estimated_sim_seconds`、`planned_length_m`、`clearance`、`map_version` 和 `start_pose`。时间使用固定本地估计模型，墙钟估计未标定时为 null。模型输出简短选择依据摘要，不请求或记录隐藏推理。

同一 1 m 邻域内，相同方向与角度不能因新 ID 或地图版本变化而重新执行；普通旋转累计绝对角度最多 π。两次有证据的 LOW_GAIN 也会阻止继续旋转。首版采用保守持久化空间记录，没有自动清除邻域封锁的启发式逻辑。

观察记录派发前、停车时、等待结束后的地图。反馈等待最多 4 秒仿真时间且最多 20 秒墙钟，预算/停车请求可提前结束。面积按网格坐标对齐，支持边界扩张和整数格原点平移；后续区域账本增加非整数原点的面积交叠诊断，但 `usable_for_trend=false`。分辨率、旋转或 frame 不兼容仍返回 UNCOMPARABLE。当前没有经核验的 SLAM processed-scan 证据，因此即使可计算地图变化，观测收益有效性仍是 FEEDBACK_INCOMPLETE，不能把相同地图判成因果 LOW_GAIN。未知→已知、已知→未知和净变化作为观测窗口指标单独报告，不声称严格因果收益；尚不能识别所有 SLAM 回环形变。

## 日志与对照入口

输出 JSON 每轮原子更新，保存源码/导航配置 SHA256，并包含完整上下文、候选、模型选择与依据摘要、request_id/task_id、本地结果、地图变化、里程、仿真/墙钟耗时、最小雷达距离、决策延迟和 token 用量。没有凭空估算货币成本。任务及地图快照在 `workspace/log/robot-skills/`，会话 watchdog 独立于 MCP 客户端运行。

```bash
./scripts/robot-skills.sh classical-loop --max-steps 3 --output logs/classical-loop.json
./scripts/robot-skills.sh classical-loop --observations --max-steps 3
```

Frontier 模式复用经典激进分数；联合模式按相同新颖性面积代理/预计仿真时间选择 MOVE 或 OBSERVE。两者共用候选、安全检查、任务仲裁和预算。连续在变化地图上分别运行不能当成严格对照；尚需相同地图、起点、初始化扫描和预算的配对重复实验，本版不宣称模型比经典策略更优。

模型决策适配器只把经枚举验证的动作/ID 转成 MCP 参数。API 使用严格结构和禁用并行工具调用，依据 [OpenAI 官方 Function calling 文档](https://developers.openai.com/api/docs/guides/function-calling)。模型结论不会更改本地任务状态。

## 本轮验收结果

相关自动测试共 **93 项通过**：任务/会话/观察去重/watchdog 25，API/MCP/循环 28，路径间距 12，观察几何/地图度量 11，原探索安全 10，执行器取消与旋转/TF 故障注入 7。

| 实测 | 本地结果 |
|---|---|
| 默认 180 s 墙钟循环 | 一个 Frontier 成功；随后 max_wall_time 终止，stopped=true |
| 最多 3 次提交，600 s 验收墙钟 / 180 s 仿真预算 | 首次候选临期，在派发前拒绝；刷新并另选后两次成功。max_steps 终止，stopped=true；共行驶 5.242 m，最近雷达 2.609 m，原始已知地图面积净增约 15.040 m² |
| 联合 MOVE / OBSERVE / STOP 两步试验 | 第一步 MOVE 成功；第二步模型看到剩余约 41 s 墙钟，主动选择 STOP。停车确认通过，没有为展示观察而强制运动 |
| 新的一步观察验收 | 模型选择固定左转 45°的 OBSERVE，真实 Spin 已开始；地图变化使剩余车体扫掠进入非自由区域，取消并确认停车。结果 aborted / rotation_path_invalidated:body_sweep_nonfree，最近雷达 2.708 m |
| 观察请求重放与参数边界 | 相同 request_id 返回原 aborted 任务，不再运动；额外 angle 参数被 MCP 拒绝，任务数不变 |

最后一项观察没有完成整个 45°转向，不能记为成功观察。该窗口还出现不对齐的地图原点变化，收益标为 UNCOMPARABLE；原始已知格面积变化不冒充可归因的观察收益。本轮已证明观察请求能进入受控执行，并在实时安全条件变化后取消停车；尚未取得完整旋转成功的现场样例或性能提升结论。

试运行暴露并修复了三个问题：临期缓存可能在模型选择后过期（保留 120 s 有效期，缓存剩余不足 90 s 时先刷新）；几何计算期间 TF 回调积压（有界清空已到达回调，仍使用原新鲜度阈值，并缓存重复视点计算）；模型 STOP 与 watchdog 抢先记录 operator_stop 的原因竞态（先记录 model_stop 再调用停车工具，旧试验记录保留）。

日志分别位于 `logs/model-loop-stage1.json`、`logs/model-loop-three-goals.json`、`logs/model-loop-observation-validation.json`、`logs/model-observation-execution.json`、`logs/observation-replay-validation.json`。这些是功能/安全验收，运行起点与地图连续变化，不是公平的经典/模型性能对照。
