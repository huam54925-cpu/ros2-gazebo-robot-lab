# 同步后端 v2

`training.envs.gazebo_backend.GazeboBackendImpl` 是一个回合一个私有容器的同步实现。`LocalDriveEnv` 负责策略观测与历史；`training.train_vehicle` 是独立车辆训练入口。原 `training.train --backend gazebo` 继续拒绝，Pendulum 的小网络和回放参数不进入车辆正式配置。

## 应用确认与时间边界

`sim_sync/SyncDrive.cc` 替换原 DiffDrive，成为左右轮唯一控制写入者。transport command 服务只排队 `(episode_id, action_id, v, w)`，服务返回的 Boolean **不是 ACK**。

暂停循环中的 PreUpdate 消费待处理命令，写入并读回 `JointVelocityCmd` 后，将 `committed=true`、应用时刻及两个轮速写入状态回执。回合和编号必须正确，命令必须在暂停状态接收；重复、跳号、跨回合或并发待处理命令均拒绝。轮半径 0.3 m，轮距 1.25 m，与当前固定模型一致。

worker 读到这个 ACK 后才发 world multi_step。每个 PreUpdate 都重新写入该动作的关节指令并验证 1 ms 步长；PostUpdate 检查命令组件已被 Physics 消费、获取实际关节位置/速度，并累计步数及首末时刻。此版本 Gazebo 10 Physics 每步会删除 JointVelocityCmd，因此不能把 PostUpdate 中组件仍存在当作成功条件。冲突的 JointForceCmd/JointVelocityReset 会使健康状态失败。

参考：[Gazebo system 阶段定义](https://gazebosim.org/api/sim/10/createsystemplugins.html)、[Gazebo Physics 命令消费和清理](https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/physics/Physics.cc)。本机运行版本为 Gazebo 10.5。

例如前帧 0.600 s：暂停 ACK applied_ns=600000000；之后正好执行 200 步；first_ns=601000000、last_ns=800000000；返回 lidar/odom/truth 时间均为 0.800 s。ACK 表示控制已写入执行器接口，不承诺车体瞬间达到目标速度；后续回执包含实际轮速。车体受惯性、接触和滑移影响，因此 odom 与真值轨迹可以不同。

## 传感器和回合隔离

世界显式固定 1 ms 物理步长。worker 使用 Decimal 验证 dt/physics_dt 整除，然后取整数，且目前只支持 0.2 s。插件逐步验证实际 dt；host 校验应用时刻、步数、首末时刻和仿真目标时间。

原世界 30 Hz odom 在 200 ms 控制周期上实测落后 2–32 ms。新插件每个物理步发布基于左右轮位置积分的 odom，速度窗口为 1 个样本。激光保留 10 Hz。缓存以整数纳秒为键，必须找到目标时间的 lidar、odom、truth；仅取 latest 是不允许的。严格 1e-6 s 协议容差保持不变。

reset 创建唯一容器名/GZ_PARTITION、全新仿真进程和缓存，插件绑定本次 episode_id。odom 消息同时带回合字段；lidar/truth 依靠本次进程和 transport 分区隔离。零动作 0 先提交，推进 600 ms 稳定，检查 odom/轮速及最后 200 ms 的真值位置和朝向漂移。stop 使用独立物理零速度，验证轮速、odom 和相邻真值姿态稳定后暂停。停止会推进仿真，之后必须 reset，不能继续沿用停止前的 Frame。

这是冷 reset，不是快速 world reset。后端错误会保留原始 InfrastructureError，附注清理错误并销毁容器；容器销毁失败时拒绝重建。close 可重复调用。

## Frame 和历史的唯一归属

schema 变为 `local-drive-v2`，不与 v1 检查点静默兼容。

- Frame.observation 只有 `lidar`、原始 SI 单位 `odom_velocity`、已编码 `goal_relative`。
- Frame.odom_pose 是原始 odom `(x,y,yaw)`，用于环境计算历史，不是真值。
- Frame.evaluation 包含必填 truth_dist、truth_pose、roll、pitch；contact=unknown。真值参考点为驱动轮轴心。
- Frame.ack 是插件执行回执，不能当策略输入。
- 环境层唯一维护 history；先更新当前动作和位移，再组装 295 维观测。局部位移由相邻 odom 差值按前一帧 yaw 旋转得到。

固定目标为初始 odom 前方 3 m；世界目标按车辆初始轴心坐标定义。不同目标和随机场景尚未实现。里程计/真值的不同用途不混用。

Gymnasium reset(seed=None) 从既有 np_random 继续抽取 Gazebo seed；显式 seed 重建 RNG。Gazebo seed 范围限定为非负 int32。时间上限为 `frame.sim_time - episode_start_time >= 120`。奖励沿用 interface.md 公式；缺失真值必须抛异常。翻倒阈值仍采用 0.8 rad 持续 0.5 s 的设计值，尚未开展专项翻车标定。

## 验收与训练

`scripts/accept_sync.py --stage all --output DIR` 验证 3 次同 seed 冷启动、真实动作执行回执、严格传感器时序、SB3 check_env、3 回合随机动作和真实 SAC Replay Buffer 故障路径。

故障注入包括：插件已提交但 worker 忽略 ACK、缓存返回旧激光、返回跨回合 Frame。实际 SB3 learn 抛 InfrastructureError 后检查 replay_buffer.size 不增加，并验证容器被销毁、后续 reset 重建。ACK 故障同时注入停车失败，检查原始错误未被覆盖。

车辆训练配置：

- `--profile formal`：原 SAC 配置，learning_starts=5000，网络 [128,128]，buffer_size=200000。少量采样不能据此声称验证梯度更新。
- `--profile acceptance --steps 64 --resume-steps 8`：显式短梯度验收，learning_starts=16、batch_size=32、buffer_size=5000；网络仍 [128,128]。检查实际更新次数、actor 参数变化、周期原子保存、模型和回放重载、预测一致以及新回合继续训练。
- `--device cpu|cuda|auto`：显式 cuda 不可用时报错，不静默退回 CPU。

车辆入口接入停车/关闭、SIGINT/SIGTERM 标志和原子检查点。保存的元数据区分 profile/schema；只加载可信且匹配的车辆检查点。短验收模型不代表导航性能或收敛。

离线镜像内包含 worker、插件、固定世界、训练源码、PyTorch CUDA 依赖和 Docker CLI。训练容器通过宿主 Docker socket 管理私有仿真容器，运行脚本不会重新构建或联网拉取镜像。

## WSL2 / RTX 4060 已验收基线

应使用训练分支记录的 [WSL2 验收](acceptance-wsl2-rtx4060.md)：Docker context 为 wsl-native，CUDA 计算与 Mesa llvmpipe EGL 渲染分别成立。离线脚本将该 context 的实际 Unix socket 挂入训练容器，设置 LOCAL_DRIVE_SIM_GPU=0，仅训练容器透传 NVIDIA GPU；不要把 CUDA 成功等同于 NVIDIA 图形渲染成功。Python3.11.16 / torch2.8.0+cu126 / SB3 2.7.0 / Gymnasium1.2.1 与现有 CUDA 锁文件一致。

只读状态服务遇到无响应时最多重试3次；非空但损坏的ACK直接失败。动作提交和world multi_step发生不确定响应时绝不重发，避免重复执行。该路径由原生Gazebo绑定环境中的 sim_sync/test_service.py 验证。

最终传输实现使用独立、持久的 `service_client.py` 进程执行原生 Gazebo transport 请求；worker 只负责订阅与同步缓存。每次物理步进不再启动 CLI。实测同一订阅 Node 上直接进行 Python request 无响应，而独立请求进程通过连续80步；这里按实测选择进程隔离，不把具体底层成因当作已证实事实。子进程IPC超时会直接失效，避免迟到响应错配到下一个请求。

`train_vehicle --profile acceptance/formal` 当前只区分配置和验收断言，不提供自动故障续训；两种profile都将InfrastructureError上抛并清理。日常实验的恢复需在学习器采样边界舍弃坏转换并开启新回合，不能在backend里悄悄reset后伪装成一次成功step。此扩展不属于当前一次性镜像验收。
