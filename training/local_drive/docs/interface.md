# 局部驾驶训练接口 v2（固定场景同步实现）

具体时序、原始 Frame 边界、冷 reset 和验收见 [同步后端](sync_backend.md)。v1 的 history 字段移至环境层唯一维护，严格时间容差不变。

## 边界与现状

策略 SAC → LocalDriveEnv → GazeboBackendImpl → 每回合独立 worker / SyncDrive 插件。
`training/envs/contracts.py` 定义原始 Frame、动作解码、激光编码和 transition 校验；应用 ACK 与物理执行回执由后端单独强校验。
`scripts/sim_probe.py` 是异步诊断工具，绝不能当训练后端。它不证明 action ACK 或精确 step。
`training.train --backend gazebo` 保持拒绝；车辆通过独立入口 `training.train_vehicle`，正式参数不继承 Pendulum 冒烟覆盖。

不启动 SLAM，不调用原 AI API，不依赖 X11，不连接实车。原 Exploration 2.0 入口保留在原仓库。

## 算法与数据

局部到点和脱困先用 SAC，MLP [128,128]；探索策略 PPO 留到后续阶段。
初始参数见 `training/configs/sac_local.json`（JSON 避免 YAML 的 off 被隐式解析为布尔值）。
这些是设计值，没有优化效果证据。

策略观测固定 float32 向量 295 维，由 LocalDriveEnv 按以下顺序拼接：

1. 90 个激光扇区，每扇区 `[最近有效距离/range_max, 有效回波比例, +inf无回波比例]`，270 维。
   无效比例为 1 减后两项；NaN、-inf、越界有限值视为无效。不能把无效数据当自由空间。
   保持原扫描角度次序（-pi 到 pi），每四束组成一个扇区。range_min/max 从 LaserScan 读取。
2. 里程计线速度 v/0.6、角速度 w/0.8，各裁剪到 [-1,1]，2 维。
3. 目标在 odom 坐标系中的距离（裁剪到20m再/20）、相对角度 sin/cos，3 维。
4. 按旧到新排列的4帧历史，每帧 `[归一化动作a0,a1, odom局部dx/0.12,dy/0.12,dyaw/0.16]`，裁剪到 [-1,1]，20 维。
   reset 清零历史。位移必须来自 odom，不来自真值。最初目标由场景同时定义 odom 目标和评价用真值目标。

策略向量按 lidar、odom_velocity、goal_relative、history 四块组装；后端 Frame.observation 仅返回前三项原始数据，history 只在环境层维护。Frame.odom_pose 另存里程计姿态；Frame.ack 是控制证据，均不作为额外策略输入。
真实世界位置、接触、真实目标距离只能进入 evaluation。
评价与目标均以驱动轮轴心为车辆参考点：当前模型轴心相对model原点的偏移为(0.5542825,0)m，真值经yaw旋转此偏移后再计算到点距离；不能混用model原点和轮轴中心。字段名校验不能证明内容来源，后端还需审查生产路径。

动作 Box(-1,1,(2,))：v=0.2+0.4*a0 m/s，w=0.8*a1 rad/s，先拒绝非有限值再裁剪。
**[0,0] 是前进 0.2 m/s，不能用作停车。** `stop()` 独立发物理零速度。
控制周期 0.2 仿真秒，时间上限 120 仿真秒（600步）。

## 后端状态机与时序

`reset(seed,scenario) -> Frame`：冷启动独立仿真进程；生成新的 episode_id；清空缓存、动作编号、历史、控制器状态。
以零命令推进稳定窗口，验证初始速度与传感器完整性，将起始时刻对齐传感器采样周期后暂停。返回 action_id=0 的同一回合观测。
相同 seed 必须生成同一场景和初态；第一版使用固定 indoor 场景和 odom 前方 3m 目标，随机化待实现；每回合冷启动稳定 600ms。

`advance(v,w,dt) -> Frame`：必须处于暂停状态；把 (episode_id,action_id,v,w) 交给仿真侧控制/步进插件。
ACK 只有在动作已应用后才能产生（收到 transport 消息不是应用成功）。
插件推进 N=dt/physics_dt 个物理步；physics_dt 必须在世界配置中显式固定且整除 dt。
返回时刻 t+dt，激光、odom、真值均须到达目标时刻，时间误差限 1e-6 秒。
原 30Hz odom 实测不满足目标时刻；新插件按 1ms 物理步发布轮式 odom，激光保留 10Hz，严格 1e-6 秒容差不变。
`validate_transition` 检查回合、动作编号、时间及输入字段；不能替代插件的 ACK 实现。

`stop()`：发送零物理速度并确认应用，推进制动/稳定步，验证 odom 与真值停止，再暂停。
`close()`：先尝试 stop，再关闭自己拥有的进程；幂等，不关闭其他用户仿真。
墙钟超时、跨回合数据、旧激光、时钟回退、ACK缺失均为 InfrastructureError；停车、记录、销毁后端，禁止向 Replay Buffer 插入该 transition。

## Gymnasium 与奖励

reset 返回 (obs,info)，step 返回 (obs,reward,terminated,truncated,info)。
到点（真值距离 <=0.35m）或翻倒/不可恢复状态为 terminated；120秒上限为 truncated。
普通接触不是直接终止；当前没有接触传感器，contact 必须标为 unknown，碰撞项权重为0。
超时基础设施故障抛异常，不伪装 terminated/truncated。

奖励窗口在reset时清空；停滞窗口未满2秒时不处罚。到达奖励仅计首次到达的终止步；reset时已在目标内应重采初态。
初始奖励：2*(d_before-d_after) -0.05*dt -0.2*stalled -0.01*||a-a_prev||² +10*arrival。
距离 d 来自真值；stalled 定义为最近2秒累计真值平面路径 <0.05m 且请求速度绝对值均值 >0.05m/s。
翻倒阈值建议 |roll|或|pitch|>0.8 rad 持续0.5秒，须实测后定版。
接触项后续必须由 contact sensor/system 事件支持，不能用激光阈值假冒。
课程：开阔到点 → 绕障/通道 → 墙角倒车恢复。训练/验证/测试按地图布局划分。

## 检查点与部署

采用 staging 目录完整写入 model.zip、replay.pkl、manifest.json，再同文件系统原子发布，再更新 latest.txt。
模型和 Replay Buffer 的 SHA256 在加载前核验；保存失败不改变上一个 latest。
该方案防进程中断发布半成品，不承诺断电持久性、不保存逐步可复现的全系统 RNG/仿真状态。
续训从新回合开始；只加载可信检查点（SB3模型/replay包含可执行反序列化内容）。
信号处理只设置标志，在正常控制流退出采样、停车、保存；车辆入口已将 stop/close 接入独立生命周期。
周期保存也必须走同一个原子发布函数，不能直接混用多个独立 latest 文件。

部署前绑定观测版本、动作范围、代码commit、镜像ID、配置哈希、地图版本、种子和测试报告。
本次 Pendulum 冒烟权重不是车辆模型，不进入 models/ 或远端发布。

## 未通过就不能开始车辆训练的门槛

- action ACK 对应真实应用、精确推进200ms、新鲜传感器对齐。
- 冷reset的新回合状态、相同seed/action序列可复核；快速reset需与冷reset对照。
- Gymnasium check_env、故障注入、终止/截断与停车路径测试。
- 随机动作数据审计；单环境通过后再实现跨容器RPC和多环境隔离。

参考：[SB3 SAC](https://stable-baselines3.readthedocs.io/en/master/modules/sac.html)、[Gazebo 10 EGL](https://gazebosim.org/api/sim/10/headless_rendering.html)。实际安装 Gazebo 10.5.0，以本机验收记录为准。
