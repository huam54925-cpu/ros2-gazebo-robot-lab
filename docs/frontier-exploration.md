# Frontier 自主探索第一版

本版本从在线 `/map` 选择观察点，通过 Nav2 `ComputePathToPose` 检查可达性，再发送单个 `NavigateToPose` 目标。探索程序不读取 SDF、历史地图文件或 Gazebo 真值，不预设路线，不直接发布速度。依赖已运行的 SLAM、Nav2、雷达和独立 `velocity_guard`。

## 启动

```bash
# 先只读检查候选，不驱动车辆
./docker/start-exploration.sh --dry-run --output /work/robot_ws/maps/frontier-preview.json

# 无预设航点探索；建议从新仿真和新 SLAM 地图开始
./docker/start-exploration.sh --initial-scan --max-goals 8 --wall-budget 1200 --goal-timeout 300 --output /work/robot_ws/maps/frontier-run.json
```

输出路径位于共享数据盘的 `workspace/maps/`，禁止覆盖既有报告。`--initial-scan` 在开始时调用带碰撞检查的 Nav2 Spin，旋转一圈以积累初始观测；仅在雷达最近间距至少 2.7 m 时允许。它不是读取预存地图或手动驾驶。原始初始快照在旋转之前记录。

一个共享文件锁阻止多个本探索程序同时运行；启动时拒绝已有活动导航目标。实验期间不要并行使用 RViz 发送导航目标或遥控。现有文件锁不是全系统任务仲裁器，未来技能层仍需统一管理所有目标来源。

## 策略与保守限制

- 将地图聚合到约 0.20 m：只有块内每个格子均为 free，粗格才算 free；不能用平均值将 unknown 变成 free。
- free 与足够大的 unknown 连通区域相邻时形成 frontier。默认未知区域至少 2 m²，边界至少 0.8 m，过滤零碎小孔洞。图像边缘按未知处理，邻接计算不跨边界回绕。
- 观察点必须在已知空地内，距未知/非空地留至少 1.9 m 的保守包络；距已知障碍至少 2.65 m，覆盖现有雷达停车圈及轮轴到雷达的偏移。距离场使用保守下界，因此实际过滤更严格。
- 每个边界最多给出三个观察位置，最多保留 12 个分散候选；先按直线距离排序，再以 Nav2 实际规划长度选择候选集中的最近可达位置，不声称找到全地图的全局最优探索目标。
- 目标发送前再检查最新地图，地图变化导致不再安全的候选不会执行。
- 本轮已访问、规划失败、执行失败或失效的位置进入空间排除记录；每个附近观察位置最多尝试一次。第一版不自动解除排除，需在新一轮任务中重新评估，避免无限重试。
- 每次到达后等候地图更新，再重新提取候选。车体尺寸和停车保护可能使部分未知区域无法接近；不为追求覆盖率降低保护阈值。

## 取消、停止与指标

预算耗尽、单目标超时或无运动进展时取消当前 action，并等待结果和速度归零；取消不能确认时尝试暂停 Nav2 生命周期，独立停车保护保持运行。Ctrl+C/SIGTERM 同样进入取消流程。不能确认停车时报告错误，不能记为成功结束。

停止原因区分：`no_frontiers`、`no_eligible_observation_goals`、`no_reachable_candidates`、`budget_exhausted`、`goal_budget_exhausted`、`operator_stopped`，以及传感器/定位错误。前两种没有候选不等于整屋探索完成；可达性结论仅针对本轮筛选后的候选集。

保存 JSON 汇总、逐事件 `.events.jsonl`、位姿与地图增长 `.trace.json`，以及初始/最终地图 `.initial.npz`、`.final.npz`。指标包括目标数、成功/失败数、排除位置、地图版本、已知面积、最小雷达间距、仿真时刻、墙钟时间和里程计积分行驶距离。该距离是轮式里程计估计，不是真值轨迹长度。没有固定可探索区域分母，因此 `coverage` 为 null，不输出未经定义的 T95 或百分比。

## 检查

```bash
python3 workspace/navigation_base/tests/test_frontier.py
```

纯地图测试覆盖全未知/封闭已知房间、边界不回绕、部分块不伪造 free、旋转地图坐标、候选排除与距离场保守性，不向实际 ROS 域注入测试消息。

## 启动建图修正

初次实测原地旋转完成，但已知栅格停留在 1,096，程序以 `no_eligible_observation_goals` 停车。这不是探索完成：SLAM Toolbox 2.10.0 的默认 `check_min_dist_and_heading_precisely: false` 在平移不足时直接丢弃扫描。现改为 `true`，保留 0.2 m / 0.2 rad 阈值，使纯旋转也能积累观测；依据上游 `shouldProcessScan` 实现：
https://github.com/SteveMacenski/slam_toolbox/blob/2.10.0/src/slam_toolbox_common.cpp

探索器同时等待 planner、controller、behavior server 和 BT navigator 生命周期进入 active。仅发现 action endpoint 不代表节点已经可执行动作。

## 实测结果

参见 [首轮实验数据与图片](experiments/2026-10-03/frontier/README.md)：4 个目标全部成功，6.28 m，516.7 s；最终没有符合安全/重复访问规则的候选而停车，整屋探索未完成。

## 激进策略对照

默认 `--policy conservative` 保持第一版候选与最近路径选择。新增 `--policy aggressive`：

```bash
./docker/start-exploration.sh --policy aggressive --initial-scan --max-goals 8 --wall-budget 1200 --goal-timeout 300 --output /work/robot_ws/maps/frontier-aggressive.json
```

从相同场景、相同起点、全新 SLAM 地图启动；两策略使用相同 Nav2、SLAM、初始 Spin、8 目标/1200 秒墙钟上限与 300 秒单目标上限。

- 候选距未知的全方向圆形限制从 1.9 m 改为 0.35 m，同时在原始分辨率地图上检查目标朝向的完整车体凸包（加 0.05 m padding 和一格对角线余量），禁止轮廓进入 unknown/occupied。这个检查只证明目标姿态；行进与转向仍由 Nav2 碰撞检查负责。
- 已知障碍保守间距仍为 2.65 m；独立 velocity guard 的阈值与超时机制不变。
- 候选间隔 0.8 m，离当前位置至少 1.2 m、距 frontier 不超过 3 m；从更大候选池选前 12 个，再由 Nav2 验证路径。
- 信息代理量：90 条射线、最远 8 m，遇已知障碍即停止；每条射线最多累计 2 m 未知深度，统计去重未知粗格面积。这是乐观可见面积估计，不是校准的熵或实际信息增益，未读取场景真值。
- 得分为 `estimated_gain / (1 + 0.12 * path_length + 0.30 * initial_turn_radians)`；规划前用直线距离筛选，规划后用实际路径长度重排。转向成本仅计起始转向，未计终点转向。
- 成功观察位置的 1 m 邻域，在至少间隔 3 个目标且全图已知面积增长至少 10 m² 后允许重访；同邻域最多两次。失败位置本轮持续排除。

这次对照同时改变候选几何、评分和重访规则，属于两套策略的比较，不能将改善单独归因于信息评分。报告同时比较完整运行和共同仿真时长，单独列出初始旋转后的新增面积；单次运行不能证明统计稳定性。

实测：[激进与保守策略对照](experiments/2026-10-03/frontier-comparison/README.md)。激进版 7/8 成功，最后一次受 guard 停车约束取消；共同时间已知面积约增加 31%，整屋探索尚未完成。
