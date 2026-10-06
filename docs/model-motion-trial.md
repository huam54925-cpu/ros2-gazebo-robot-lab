# 模型运动接口：一次受限仿真测试

> 历史版本记录：本文的单步/候选决策入口已退役。当前使用 `scripts/robot-investigate.sh`，见[主运行路径整理记录](runtime-consolidation-2026-10-06.md)。

在只读链路上增加显式启动的运动 MCP 服务。模型通过 `move_forward_trial(request_id)` 请求一次移动；本地执行器根据当前地图位姿，生成**沿当前朝向前方 0.4 m** 的目标，再交给 Nav2。

后续升级见[Robot Skills 与安全 Frontier MCP](robot-skills-mcp.md)。固定前进的新请求已使用统一任务账本、仲裁、停车锁及监督执行器，旧请求保留历史重放语义。

这是小范围仿真接口，尚不是任意目标导航或长期自主探索接口。普通 `robot-status.sh` 继续只暴露只读工具；只有下面的专用命令启动运动工具。

2026-10-04 实测：本地试验行驶 0.266 m 并停车；行驶中取消试验在 0.021 m 后停车；模型 `gpt-6.1-sol` 发起的试验行驶 0.256 m 并停车，最近雷达距离 4.132 m。同 ID 重放只返回原结果，没有重发目标。

```bash
cd /mnt/robot_disk/ros_sim
./scripts/robot-motion-trial.sh --model
```

每次执行该命令授权一次新的试验，可能让仿真小车实际移动。模型使用 `.env` 中的配置；每次最多调用两次 OpenAI API，一次请求工具、一次解释结果。没有后台模型循环。

## 限制与保护

- 工具不接受坐标、速度、保护阈值或 shell 参数，只接受标准 UUID；固定请求距离 0.4 m，同一 MCP 会话最多一个不同请求。
- Nav2 规划器沿用 `allow_unknown: false`、碰撞与 footprint 配置。额外复用整条路径雷达间距检查，包含 TF 偏置、朝向变化、最终转向和地图更新；保护阈值 1.9 m，规划要求 2.15 m。
- 规划路径超过 0.8 m 就拒绝；执行累计里程超过 0.65 m、目标运行超过 45 秒、传感器/定位过期、guard 触发或路径失效即取消。0.65 m 是触发取消的预算，不是制动距离的硬保证。
- 专用行为树只有计算路径和跟随路径，不自动旋转、倒车恢复或反复重试。
- 与 Frontier 共用导航所有者文件锁，拒绝已发现的活动导航任务。外部 RViz/ROS 程序不受文件锁控制，本轮试验期间不要同时从其他客户端发目标。
- 相同请求 ID 只返回既有结果，不重新执行；结果缺失或前次停车未确认时拒绝下一次试验。
- 取消等待 Nav2 确认，再用新鲜里程计验证速度稳定低于 0.02；失败时暂停导航生命周期。执行器超时/异常且可能已发目标时，宿主机监督逻辑可停止 `robot-nav2`；独立 guard 仍在仿真容器内运行。

模型不能放宽这些限制。固定试验通过统一框架仲裁，仍不是完整的自主运行系统，也尚未完成任意进程故障、动态障碍与真实硬件的专项验收。

## 结果与取消

终端首先显示 `request_id`；结果默认写到 `logs/motion-trial-<request_id>.json`，原始动作、规划、里程计轨迹在 `workspace/log/motion-trials/`。

另开一个终端取消本次试验：

```bash
./scripts/robot-motion-trial.sh --cancel <request_id>
```

`cancel_requested` 仅表示已请求取消，必须等任务最终结果 `stopped: true` 才表示停车已确认。需要重复查询同一请求而不重发运动，可以运行：

```bash
./scripts/robot-motion-trial.sh --request-id <原来的request_id>
```

Nav2 当前位置容差是 0.15 m，因此目标前移 0.4 m 时，实际位移可能约为 0.25–0.4 m；报告保留请求距离和真实里程，不把请求值写成实测值。

完整关闭当前仿真服务：

```bash
docker stop robot-nav2 robot-sim-gui
```

模型工具调用依据 [OpenAI 官方 Function calling 文档](https://developers.openai.com/api/docs/guides/function-calling)。模型输出只用于请求和解释，运动成功/取消/停车状态以本地执行结果为准。
