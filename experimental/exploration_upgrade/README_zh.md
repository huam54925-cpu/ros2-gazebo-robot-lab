# 未知空间语义与防循环探索：增量改造指南

版本：0.1.0 · 2026-10-04 · 面向 ros2-gazebo-robot-lab

## 0. 交付范围与证据边界

本包提供离线 Python 辅助模块、合成演示和回归测试，不是已部署的 ROS/MCP 补丁。它不会启动机器人、调用模型、发布速度、发送 Nav2 goal、修改 SLAM 地图、改变 guard 或解除停车锁。`retain_for_safety_recheck=True` 只表示“可以进入你原来的安全复核”，不等于安全或允许运动。

本指南结合了三类材料：

- 你上传的 `robot-skills-mcp.md` 与 `robot-skills-validation.json`：已有六个工具、候选 ID 执行、幂等账本、路径复核、停车锁、1.9 m guard / 2.15 m 规划要求，单次模型选点闭环和 57 项既有测试。这些是你提供的本地验收记录，不是本包重新运行的结果。
- 本次读取的远端提交 `fffe604d1f16a46a11acac30ea80c5b5215eb3b3`：`workspace/navigation_base/exploration/frontier.py` 与 `aggressive.py`。远端代码不一定包含你今天尚未推送的本地修复，因此不能直接用远端旧文件覆盖本地。
- ROS/Nav2 官方文档：用于核对 costmap、未知空间和雷达清除参数。具体部署仍须以你容器里实际节点、插件和参数为准。

本轮没有收到导致徘徊的完整连续决策日志，不能仅凭描述确定根因。这里给出可区分根因的诊断与改造方法。65 项通过是本包的离线逻辑测试，与原工程 57 项分开统计，不代表机器人在线安全或整屋探索通过。

## 1. 先修正两个过度简化的做法

### 1.1 路径失败不是目标区域全坏

若机器人从西侧前往目标时在中途被挡，只能说明这个接近路径当前不合格。目标仍可能从东侧到达。不要在目标附近随意画一个 0.8 m 的“永久障碍圆”。

应记录失败范围：目标姿态、接近路径/通道、传感器/系统，分别处理。只有目标 footprint 的具体几何证据失败，才对相近目标姿态暂时抑制。路径失败抑制同一接近方向或通道，不封禁所有抵达方式。

### 1.2 回程不是振荡

`A→B→A` 可能是探索支路后的必要回程。不要把重复经过的地方写入 Nav2 障碍图，也不要禁止路径穿过已访问区域。

先检测：四次已完成的观察目标呈 `A→B→A→B`，各次已确认停车，地图增益可比较且总新增信息很低，重复位置附近没有相关新证据。该规则只约束“再次选这里作为观察目标”，不约束经过这里去远处、返航或合法中转。

### 1.3 未知原因不是地图中的真实标签

`unknown_open / occluded / out_of_range` 只能是带证据的推测。单张 OccupancyGrid 无法可靠恢复每格变成 unknown 的历史原因。本版不伪造这些标签，返回 `unknown_cause=not_identified_from_map_alone`，并报告射线因已知障碍、未确定占用、地图边界、量程而终止的统计。

## 2. 当前代码已有的东西，不要重复推倒

远端 `frontier.py` 已从原始数据构造 free、occupied，并提取 free/unknown 边界。它有候选排除半径、最小目标距离、候选数量上限等过滤。

远端 `aggressive.py` 已有按朝向的 footprint 检查。`gain()` 已使用射线估计，遇到已知墙体终止，并将未知穿透深度限制为 2 m；这不是“完全没有遮挡处理”。它也明确把收益叫 optimistic proxy。粗预筛还存在固定的 obstacle_clearance=2.65 m；你本地新增的逐姿态 2.15 m 检查与它不一定等价，必须记录是否在更早的保守预筛中被拦下，不能看到“拒绝”就认定 unknown 被当成实体墙。优化早期预筛也必须以回放对照验证，不能靠降低 guard。

值得核查和增量改进的是：

1. 粗网格的 `unknown=~(free|occupied)` 同时包含真正未观测、混合粗块和中间占用概率。对保守可通行过滤可以这样处理，但不能把所有这些都当作“全新未观测面积”奖励。
2. `gain(x,y)` 的远端接口从候选底盘参考位置发射射线，未将 yaw 与真实雷达偏置一起传入。你已在安全检查中修复偏置，信息收益估计也应使用同一套正确 TF。
3. 最大候选数 12，且预评分后截断；远处的新区域是否在交给模型之前已经被截掉，必须有诊断。模型无法选择未提供的候选。
4. 失败排除与访问排除是否跨多轮真正持久化、是否被批次 ID 或 map hash 更新绕过，要从本地最新代码和事件记录确认。

## 3. 目标架构：不变的是安全执行层

```text
原始 /map + 当前 TF/传感器证据
  → 地图语义与 Frontier 诊断
  → 现有观察姿态生成器
  → 原有收益代理 + 新的可见未知边界代理
  → 历史失败/低收益观察过滤
  → 原有 footprint / Nav2 / 整路径雷达间距复核
  → 当前允许候选 ID 集合
  → AI 选择 ID 或停止
  → 原任务账本、仲裁、执行前再检查
  → Nav2 → 独立 guard → 权威终态与停车确认
  → 本地写入一次探索事件
```

`guard=1.9 m` 和规划要求 `2.15 m` 保持不变。新模块不替代 footprint、动态避障、传感器新鲜度、任务仲裁、停车验证。

保留六个 MCP 工具，先只增加返回字段和本地筛选逻辑。不要在这个改造中同时开放任意坐标、任意转角、速度、shell、resume、recover。

## 4. 第一阶段：只增加诊断，不改变选点行为

每轮保存同一时刻的原始地图、地图 metadata、位姿/TF、候选生成参数、全部候选及每级拒绝原因。先使用“影子模式”：新模块只计算和记日志，旧执行器仍做唯一决策。

建议每个候选保存：

```json
{
  "frontier_id": "保留原来的批次ID",
  "stage": "path_clearance",
  "reason": "PATH_GUARD_CLEARANCE",
  "raw_target_state": "free",
  "raw_unknown_cells_under_footprint": 0,
  "raw_occupied_cells_under_footprint": 0,
  "planning_clearance_required_m": 2.15,
  "predicted_min_clearance_m": 1.88,
  "failure_scope": "approach",
  "failure_witness_map_xy": [4.1, 2.8],
  "approach_key": "由本地通道或失败截面产生的稳定键",
  "model_was_offered_this_candidate": false
}
```

这是字段示意，不是本次真实数据。

至少区分以下原因：

| 原因 | 能说明什么 | 后续处理 |
|---|---|---|
| GOAL_FOOTPRINT_OCCUPIED | 目标姿态覆盖已知障碍 | 在 free 区域换观察姿态 |
| GOAL_FOOTPRINT_UNKNOWN | 目标姿态包含未观测区域 | 后退到已知自由区域观察，不把 unknown 改成 free |
| GOAL_FOOTPRINT_UNCERTAIN | 已有观测但未满足 free 判据 | 等新证据或换观察点，不能冒充全新 unknown |
| PATH_UNKNOWN | 该路径需要穿越未观测区域 | 当前拒绝；寻找另一条已知安全路径 |
| PATH_GUARD_CLEARANCE | 沿途某一处雷达间距不满足 | 记录路径见证，换路径/目标 |
| NO_NAV2_PATH | 本次规划未给出路径 | 保留原始错误；不直接认定目标后面是墙 |
| PATH_BUDGET_EXCEEDED | 超出这轮运动试验预算 | 区别于障碍；扩大候选搜索不自动放大执行预算 |
| RECENT_LOW_GAIN_OBSERVATION | 同一观察目标近期信息很少 | 暂不重复；通行路径仍可经过 |
| SENSOR_STALE / LOCALIZATION_STALE | 数据不满足运动条件 | 进入本地安全处理，不写几何禁区 |
| STOP_LATCHED / INDETERMINATE | 运动被锁或状态未解决 | 保留现有人工恢复边界 |

同时保存各级“输入数量、保留数量、拒绝数量”。要看清楚 unknown 是在哪一级被拒绝：原图、粗化、footprint、costmap 膨胀、Nav2、路径间距，还是历史策略。

只读查看本地位置：

```bash
cd /mnt/robot_disk/ros_sim
git status --short
git rev-parse HEAD
rg -n 'allow_unknown|track_unknown_space|inflate_unknown|inflate_around_unknown' workspace
rg -n 'unknown_clearance|obstacle_clearance|maximum_candidates|estimated_gain|excluded' workspace scripts
```

`rg` 未安装时可用 `grep -R -n -E`。不运行 `git reset`、不覆盖旧文件、不清空任务数据库。

在已有 ROS 容器中可只读列节点和 dump 实际参数；节点名以 `ros2 node list` 输出为准。例如旧环境使用 `robot-nav2` 时：

```bash
docker exec robot-nav2 bash -lc 'source /opt/ros/lyrical/setup.bash; ros2 node list'
```

不要把 `ros2 param set` 放进诊断脚本。参数配置文件和运行时参数可能不同。

## 5. 第二阶段：分开原图语义与导航成本

本包沿用被检查的项目阈值，不声称这是所有 ROS 地图的统一规定：

```python
raw = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
unknown = raw == -1
free = (raw >= 0) & (raw < 25)
occupied = raw >= 65
uncertain = ~(unknown | free | occupied)
```

`uncertain` 不是“从来没扫描过”，也不是“确认实体障碍”；导航层仍可以拒绝通过它。

不要用 `uint8` 提前读取原图：负数 unknown 会丢失符号。Nav2 的内部 costmap 使用另一套成本编码，也不能直接套上述 0..100 占用阈值。即使一个 costmap 被发布为 OccupancyGrid，它也可能表达导航成本而非原始 SLAM 占用语义；必须确认输入 topic 的来源。

原则是：

- 原始 SLAM map：回答观测和占用情况。
- 导航 costmap：回答运动代价、膨胀与规划限制。
- 策略记忆：回答这个动作近期是否不值得重试。

三者不能互相覆写。

继续保留现有 `allow_unknown:false`。核查 `track_unknown_space`：Nav2 文档说明 false 会把未知按 free 处理，不是“更谨慎”。核查 unknown 相关膨胀设置，但不要为消除拒绝就盲目关闭它们；需在小场景中分别测原图、footprint、路径和 guard 的一致性。

对雷达 `+inf`、NaN、量程边界分别统计。无回波不等于扫描端点有障碍，也不能未经驱动语义验证就认定整条射线自由。Nav2 的 `inf_is_valid`、`clearing`、`raytrace_max_range` 属于 costmap 处理；SLAM 是否把对应方向写入 /map 要另行检查。不要用大模型或新模块伪造 free 栅格。

## 6. 第三阶段：观察姿态和信息收益

### 6.1 不向 unknown 直接发目标

现有候选生成器继续在 free 中采样，检查真实朝向 footprint，并且目标需要能观察某片未知边界。目标不是 Frontier 的几何中心，更不是墙后 unknown 的中心。

目标本身 free 只是一项必要条件；完整车体和整条路径仍由原模块判断。

### 6.2 从真正的雷达位置计算视线

设底盘地图位姿为 `(x,y,theta)`，雷达在底盘系偏置为 `(dx,dy,dtheta)`，那么：

```text
lidar_x = x + cos(theta)*dx - sin(theta)*dy
lidar_y = y + sin(theta)*dx + cos(theta)*dy
lidar_yaw = theta + dtheta
```

当前项目记录出现过约 `(-0.7057095, 0)` 的底盘到雷达水平偏置，但接入时应读取你已校验的 TF，不把文档中的数值永久写死。

### 6.3 新代理量先并行记录，不立刻替换原评分

`visible_unknown_boundary()` 对每条射线只记第一个 unknown 栅格。已知障碍和 uncertain 都阻断后续射线；不会透过第一层 unknown 保证后面都能看见。使用 supercover DDA，避免简单大步采样跳过一格厚的墙。

返回 `proxy_m2` 是“采样射线遇到的唯一未知边界栅格数 × 单格面积”，不是预测可见总面积，不是熵，不是保证的新增面积。它受地图分辨率、射线数和离散化影响，不能与原来的“向 unknown 内深入 2 m”的面积代理直接按同一数值阈值替换。

`enrich_existing_candidates()` 还计算候选与当前雷达位置的未知边界集合差。这能减少“当前已经能够观测的方向还被反复奖励”，但只是辅助代理；移动可能揭示同一未知边界后方的新空间，所以差集为零不能单独证明动作没价值。

地图数组外部只计 `rays_leave_map`，不虚构外部未知面积。若大量射线直接离开数组，应检查地图扩展/更新与边界候选，不能因为本代理为零就宣布探索完成。

使用方法：

```python
from exploration_support.grid import Grid
from exploration_support.adapter import enrich_existing_candidates

# raw_map 是 /map 的二维有符号数组；origin_yaw 必须从 quaternion 求得。
grid = Grid(raw_map, resolution, (origin_x, origin_y, origin_yaw), frame="map")
enriched = enrich_existing_candidates(
    grid=grid,
    robot_base_pose=current_map_pose,
    candidates=existing_candidate_dicts,
    base_to_lidar=verified_base_to_lidar_tf,
    range_m=effective_mapping_range_m,
    rays=180,
)
```

此处变量由现有 ROS 状态桥提供；本包的合成演示无需这些外部变量。`effective_mapping_range_m` 以实际传感器和建图配置核对，不因雷达名义 12 m 就假定整条处理链都用满 12 m。

## 7. 第四阶段：持久化探索记忆

`EventStore` 是辅助数据库，不替代 `tasks.sqlite3`。初次可放到：

```text
workspace/log/exploration-memory/events.sqlite3
```

本包默认每次读写显式关闭连接，附有 1,200 次读取描述符检查。现有执行器仍是运动结果和停车状态的唯一权威。

每个真实终态写入一次：

```python
from exploration_support.memory import Event, EventStore
from exploration_support.grid import capture_patch

store = EventStore("workspace/log/exploration-memory/events.sqlite3")
event = Event(
    event_id=f"terminal:{task_id}",
    mission_id=mission_id,
    map_epoch=map_epoch,
    step=decision_sequence,
    target=actual_final_map_pose,   # 成功时用实际观测位姿；不是请求距离假装实测
    status=local_result["status"],
    stopped=local_result["stopped"],
    gain_m2=comparable_measured_gain_or_none,
    patch=capture_patch(map_after, actual_final_map_pose[:2],
                        radius_m=observation_evidence_radius_m,
                        spacing_m=0.25),
)
store.append(event)
```

同 event_id、相同内容返回“不新增”；不同内容产生冲突。MCP 重放、查询任务状态和模型重新解释都不能额外计一次访问。

成功观察的 patch 应覆盖与该观察收益相关的区域；路径失败的 patch 应围绕实际最小间距见证/受阻截面。代码中的 1.5 m 默认仅为小型演示参数，不能保证覆盖你的 2.15 m 间距影响范围；本地应按 footprint、雷达偏置和失败路径走廊构造证据范围。证据范围大并不意味着把那片区域设为禁行区。

### 7.1 稳定身份

临时候选 ID 用于执行协议，空间/任务身份用于记忆，两者分开。`F_batch1_2` 换成 `F_batch2_5` 不应消除“这个观察位姿刚去过且没有收益”的历史。

使用：mission_id、map_epoch、世界坐标、朝向、局部语义 patch；不要使用会随地图数组扩容改变的行列号作为永久空间键。

`map_epoch` 表示同一几何参照与可比较的 SLAM 地图阶段，不是每帧 map hash。发生地图重载、坐标系重置、显著回环形变时，暂停选点并显式重投影/失效相关记忆；不能凭同名 `map` 假定物理位置始终一致。重置探索记忆绝不能解除停车锁或原任务幂等记录。

### 7.2 失败类型

- `goal_pose`：目标姿态 footprint 冲突；抑制相近位置与朝向，允许换观察姿态再验证。
- `approach`：某条接近路径的间距或连通问题；必须给出稳定通道/受阻截面的 `approach_key`。另一条入口不受该条记忆封禁，但仍需完整复核。
- `system`：过期传感器、定位丢失、未知执行状态等；由现有 supervisor 处理，不涂到地图上。

`approach_key` 不是新生成的 frontier ID。可以从本地通道图或“受阻截面位置 + 接近方向”获得。简单量化坐标仍可能在边界抖动，应结合近邻匹配；本包不声称实现了完整拓扑通道关联器。没有足够路径证据时，不建立宽泛区域禁区，只保留明确失败记录及重试预算。

### 7.3 何时允许再检查

全局 map_version 变化不是重新放行的充分条件。比较固定世界采样点的局部语义变化。记忆比较把“数组外”和“数组内 -1”都视作未观测，避免单纯扩容错误释放记忆；诊断和射线统计仍区分二者。

本包示例阈值为至少 3 个样本且至少 5% 改变。这是去除微小变化的起点，不是校准结论。变化只能获得“重新规划资格”，不能绕过安全检查。

不要仅靠时间到期就反复重新执行静态危险路径。时间到期可以触发一次只读复核；若局部证据仍相同且仍不合格，继续抑制。对未写入地图的动态障碍，需要新鲜扫描与原局部规划检查，而不是伪造地图变化。

## 8. 第五阶段：防止两点决策循环

`low_gain_cycle()` 只分析最近四个真实完成的观察任务。任一项是中转、返航、失败、未确认停车、无法比较收益，均不把它当作满足条件的 ABAB。

`filter_for_recheck()` 输出建议而不是运动命令：

```python
from exploration_support.policy import Candidate, Config, filter_for_recheck

history = store.events(mission_id, map_epoch)
advice = filter_for_recheck(
    grid=current_grid,
    candidates=candidate_objects,
    events=history,
    mission_id=mission_id,
    map_epoch=map_epoch,
    step=decision_sequence,
    cfg=Config(),
)
kept_ids = {a.frontier_id for a in advice if a.retain_for_safety_recheck}
# 仍必须交给现有 footprint / Nav2 / 整路径 guard 一致性检查。
```

注意边界：这些抑制只影响“选这个位置作为下一观察目标”。不能禁止 Nav2 经过旧位置，也不能让模型把某动作自行标成 transit 来绕过策略。`purpose` 来自本地任务管理器，不是模型自由参数。

默认 0.5 m 位置邻近、45°朝向邻近、5 个决策序列、0.5 m² 低收益阈值都只是可调启发式。与 1.9 m / 2.15 m 安全约束完全分开。

### 没有剩余候选时

不要退回一个刚排除的点凑数。按顺序：

1. 只读刷新传感器与地图状态；缺失数据不是“零收益”。
2. 扩大候选检索：保留不同 Frontier 簇、不同空间区域的代表，不只取相同局部的前 12 个高分点。
3. 在当前允许地图与执行预算内寻找其他观察点或合法路径。通过旧位置去新区域可以允许。
4. 仍没有时停车/保持停车，并报告 `NO_SAFE_INFORMATIVE_CANDIDATE`、`CANDIDATE_SEARCH_EXHAUSTED` 或 `BUDGET_EXHAUSTED`。

扩大检索范围不等于自动提高 8 m 候选路径、180 秒单目标、10 m 累计里程等现有执行限制。超预算是独立原因；长途分段能力须另做技能设计和验收。

## 9. MCP 接入位置与原子性

`get_robot_state()` 不变。`get_decision_context()` 继续只读，不应因“读上下文”刷新规划或解除锁。增加字段：

```json
{
  "decision_mode": "LOCAL_OR_EXPAND",
  "low_gain_cycle_detected": true,
  "candidate_filter_counts": {},
  "recent_action_summaries": [],
  "candidate_search_scope": "whole_current_map",
  "candidate_search_truncated": false,
  "remaining_unknown_is_not_completion_proof": true
}
```

`get_safe_frontiers()` 仍负责本地候选刷新，在日志中区分“几何安全”和“当前策略允许”。将通过所有阶段的候选提供给模型；保留所有被过滤候选的原因供诊断，但不要让被过滤候选仍可执行。

`execute_frontier(frontier_id, request_id)` 保持两个参数。执行前：

```text
先处理原有 request_id 幂等重放/冲突
  → 对新请求确认无未解决任务、无停车锁、授权与预算有效
  → 校验候选批次与有效期
  → 根据最新探索历史再次检查是否允许选择
  → 原有当前地图重新规划与整路径复核
  → 原有并发仲裁、提交与停车优先级规则
```

不要长时间持有 SQLite 写锁进行规划或 API 调用。选点、检查、提交之间地图/停车锁可能改变，要保留你原来在最终提交处的再验证和仲裁。另一线程的停车优先级不能被新层覆盖。

`get_task_status()` 不变。任务终态后由本地监督逻辑写探索 Event；不要由模型写 gain、stopped 或地图变化结论。

`stop_robot()` 不变，仍须等待权威停车确认。模型断开/超时按照你已有 supervisor 规则处理，不能把网络断开解释成已经取消。任何未解决 indeterminate / stop_unconfirmed 都阻止下一轮运动。

本包 `parse_choice()` 仅演示可选客户端选择格式；不是要求修改现有 MCP schema。已经通过原生工具调用选择 ID 的客户端，不必额外叠加它。

## 10. 实际新增面积怎样算

不要用图片分辨率变大、map.width×height 增大、unknown 数减少的未经对齐差值冒充观测收益。

`observed_delta(before,after,same_map_epoch=True)` 统计旧 unknown/旧数组外 → 新非负占用值，也单独统计已知信息丢失。支持相同分辨率、相同 yaw、整数格原点平移/扩容。非整数平移、不同 frame、不同几何阶段会报错，接入层将 gain 记为 null，而不是 0。

即便数组能对齐，回环优化也可能改变内部结构。调用者须保证 `same_map_epoch` 的真实性；模块不能自动检测所有 SLAM 非刚性变形。

长期度量建议至少保留：新增观测面积、丢失面积、净已知面积、实际里程、仿真/墙钟耗时、候选拒绝原因、模型延迟与成本、停车原因。没有固定评估区域分母时不报告 coverage。静止时也可能有 SLAM 延迟更新，应保存同样等待窗口的基线，不能把所有增量都归功于 AI 选择。

## 11. 运行本包

在解压出的 `exploration_upgrade` 目录中，Python 3.10+：

```bash
python3 -c 'import numpy; print(numpy.__version__)'
python3 -m unittest discover -s tests -v
python3 -m exploration_support.demo
```

缺少 numpy 时在数据盘建立虚拟环境，不修改系统 Python：

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python -m exploration_support.demo
```

`requirements.txt` 给的是兼容范围；发布你的实验时冻结实际版本并记录环境。这里验证环境和结果见 `validation/offline-validation.json`。

建议先解压到数据盘独立目录，例如：

```text
/mnt/robot_disk/ros_sim/experimental/exploration_upgrade/
```

不要直接覆盖 `frontier.py`、`aggressive.py`、guard、账本或本地未推送文件。本包不含自动修改这些文件的脚本。

目录：

```text
exploration_support/
  grid.py       原图语义、射线代理、局部证据、面积变化
  adapter.py    给现有候选只读添加指标，保留原 ID 与旧评分
  memory.py     逐事件幂等、显式关闭连接的辅助数据库
  policy.py     路径/目标分域过滤、低收益 ABAB、选择格式检查
  demo.py       全离线合成演示
 tests/         65 项测试
 validation/    本次真实离线输出
```

## 12. 验收顺序

### 第一步：只读诊断

保存一次完整候选生成链。确认远方候选是否在截断前存在，确认失败发生在哪一级。只读新增字段不应改变原始候选集合或旧执行结果。

### 第二步：离线测试与旧记录回放

运行本包测试。再用你归档地图和路径复现已知失败；合成测试不能代替旧第八段回放，更不能代替在线复现。需要原始地图、TF、路径、配置和失败见证，单个汇总 JSON 不足以复原完整运动。

### 第三步：针对性小场景

| 场景 | 应验收的行为 |
|---|---|
| 已知墙后大片 unknown | 不把墙后体积全部计入可见收益，不穿墙 |
| 已知自由走廊接 unknown | 选 free 中的观察姿态，扫描后地图可继续扩展 |
| 同目标一侧失败、另一侧可达 | 只抑制失败接近方式，保留另一条合法路径 |
| 两点反复且面积无增长 | 换 ID 也能识别，最终选择新观察点或明确停车 |
| 经过旧点到新区域 | 不被“访问禁区”切断合法通行路径 |
| 地图远处一格变化 | 不自动释放无关失败记忆 |
| 地图原点扩容 | 不重复奖励原已知区域，不遗失空间身份 |
| 候选仅超出本轮路径预算 | 报预算限制，不报实体障碍 |
| 地图/TF过期或停车未确认 | 不启动下一轮 |
| 参数注入/旧 ID/同请求重放 | 保留原有拒绝和幂等行为 |

### 第四步：三步受限连续闭环

先只允许最多三个新运动任务；模型非法选择最多修正一次，仍无效则保持停车。所有被拒绝的选择不应无限刷新候选或无限消耗 API。保留运动、时间、尝试、规划与模型调用总预算。

这一步不是“多开自由度”：不增加任意坐标、observe 或自动解锁。未知信息获取仍通过已有 Frontier 到达后自然扫描完成。

### 第五步：对照

先固定场景、起点、初始地图、初始扫描、传感器/导航/guard、预算与资源，重复原经典策略、新几何/记忆策略、相同新候选上的模型策略。区分几何候选改进和模型选择的贡献。

完成条件是“有新候选时不无收益往返；无安全候选时能解释停止”，不是“无论如何都跑完整屋”。真实环境真值只进入评估，不传给在线决策。

## 13. 给本地开发代理的任务说明

在现有 /mnt/robot_disk/ros_sim 增量改造，不重建工程、不改 1.9 m guard、不降低 2.15 m 规划要求，不开放任意坐标/速度/恢复锁接口。

先核对本地最新候选生成器、Robot Skills 与任务 supervisor。添加每级候选拒绝诊断；区分原始 unknown、uncertain、occupied、costmap 导航成本与策略禁选。保留现有射线估计，并在同一地图快照上并行增加基于真实雷达 TF 的第一未知边界代理，先不替换旧评分。

添加跨轮探索事件记忆。事件从本地权威终态产生且 task/event 幂等；区分目标姿态失败、接近路径失败和系统失败，不把路径失败涂为目标周围障碍。不因新候选 ID 或无关 map hash 改变清空失败。对有局部新证据的候选仅重新授予复核资格。

检测已完成观察任务的低收益 ABAB，不禁止合法回程、中转或经过旧位置。候选用尽时扩大只读搜索并保留区域多样性；不回退执行被拒绝候选，不自动扩大运动预算。

保留六个 MCP 工具的运动参数限制、现有幂等账本、停车优先级、执行前后检查。所有新逻辑先做影子模式和离线回放，原 57 项回归继续跑；本包离线测试另行统计。接入完成后先做最多三目标仿真，输出改变的文件、参数、测试日志、每步决策与停止原因；不把离线通过宣称为在线安全验收。

## 14. 参考来源

用户材料：`robot-skills-mcp.md`（第 18–49、89–108 行），`robot-skills-validation.json`。远端读取到的代码版本仅用于给出接入位置，不代表本地 HEAD。

官方与代码引用地址（核对日期 2026-10-04）：

```text
https://github.com/huam54925-cpu/ros2-gazebo-robot-lab/blob/fffe604d1f16a46a11acac30ea80c5b5215eb3b3/workspace/navigation_base/exploration/frontier.py
https://github.com/huam54925-cpu/ros2-gazebo-robot-lab/blob/fffe604d1f16a46a11acac30ea80c5b5215eb3b3/workspace/navigation_base/exploration/aggressive.py
https://docs.nav2.org/lyrical/configuration_and_development/configuration_guide/core_servers/costmap_2d/
https://docs.nav2.org/lyrical/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/
https://docs.nav2.org/lyrical/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/
```

本包的新算法、阈值和测试是这次提供的实现方案，不应写成上述来源已经验证的结果。它还没有经过你的实时地图、ROS 调度、真实动力学或故障注入整体验收。
