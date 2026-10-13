# 固定场景导航评估

`training.evaluate_vehicle` 只加载已核验的车辆模型并调用 `predict(..., deterministic=True)`。不执行 `learn()`，不反序列化 Replay Buffer，不改写检查点。用于给同一固定场景建立可复测的导航质量基线。

## 执行

在 `training/local_drive` 目录、已有离线 v2 镜像和 NVIDIA CUDA 支持的 Linux/WSL 环境中：

```bash
DOCKER_CONTEXT=wsl-native bash scripts/evaluate-vehicle.sh \
  /absolute/path/to/training/checkpoints \
  /absolute/path/to/new-evaluation-output \
  --seeds 7 17 23 --max-steps 600
```

第一参数可以是含 `latest.txt` 的检查点根目录，也可以是含 `manifest.json` 的具体 `step-*` 目录。输出目录必须不存在。脚本冻结本次评估源码，挂载检查点为只读，用容器中原有的 `LocalDriveEnv` 和同步后端运行，每回合独立冷启动。

原离线镜像不包含新增评估模块，因此脚本仅把 `evaluate_vehicle.py` 挂载进训练镜像；不替换环境或仿真插件，不需要重建镜像。默认 CUDA 推理，仿真沿用已验收的 Mesa llvmpipe 软件 EGL。脚本记录训练镜像 inspect；报告记录仿真镜像 ID、实际环境/后端/评估源码哈希，以及模型和回放的前后哈希。

输出包括 `evaluation.log`、源码快照、镜像记录，以及 `evaluation/evaluation.json` 和每回合 `episode-*.jsonl`。逐步记录动作、奖励、真值/里程计姿态、目标距离、时间戳、动作回执和终止标记；每步刷新到磁盘，故障前的有效轨迹仍可检查。

运行完成后可独立重算指标，并检查全部有效转换的回合/动作编号、200 个物理步、ACK 和传感器时间：

```bash
python3 scripts/verify_evaluation.py /absolute/path/to/new-evaluation-output/evaluation
```

## 指标定义

| 指标 | 定义 |
| --- | --- |
| 到点率 | 到点回合数 / 请求评估回合数；沿用环境阈值：驱动轮轴心真值距离 ≤ 0.35 m，且环境返回终止 |
| 路径长度 | 相邻有效控制步的真值 XY 位移之和，单位 m；包含最后一个返回的控制步，不含 reset 稳定与停车清理过程 |
| 超时率 | 环境达到 120 秒仿真时限、没有同时到点或其他终止的回合数 / 请求回合数 |
| 回合奖励 | 成功返回的每步奖励直接相加，不折扣；汇总报告其均值 |
| 最小/最终距离 | 回合采样期间最近及最后一帧的真值目标距离 |

每步 0.2 秒，默认 600 步覆盖完整 120 秒回合。在最后一步到点同时触及时限时，计为到点。其他环境终止计为 `terminal_failure`；当前环境对应持续翻倒等终止逻辑。若主动设定更小的 `--max-steps`，提前截断记为 `evaluation_limit`，不冒充环境超时。

路径是 5 Hz 控制帧上的折线长度，不是每个 1 ms 物理步的积分；快速弯曲运动可能存在采样误差。模型只有激光/里程计等策略观测，评价所用真值不会加入策略输入。

## 故障与适用范围

基础设施错误、评估代码错误、停止信号或容器清理错误均保留为独立结果。任何回合未完整评估时，整次报告为 `incomplete`，总体到点率、超时率和均值为 `null`，进程非零退出；不会把缺失数据当成模型导航失败。评估不自动重新发送动作，不自动续跑失败回合。

当前只有 `indoor_fixed` 地图、固定初态与初始前方 3 m 目标。三个 seed 是同场景基线的重复试验，不是三个不同导航任务，不能据此声称跨场景泛化。尚无接触传感器碰撞指标；到点率不能代替碰撞率。

这是可单独调用的评估入口，尚未自动接入正式训练的周期回调，也没有实现长期训练故障恢复。
