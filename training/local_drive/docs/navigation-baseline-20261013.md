# 固定场景导航基线 · RTX 4060 · 2026-10-13

已用现有第 72 步检查点完成三回合确定性导航评估。评估执行完整结束，无基础设施或清理错误；模型在本次固定场景的到点率为 **0%**，超时率为 **100%**。这将作为后续训练的质量基线，不把“程序能训练”解释为“已经学会导航”。

## 四项指标

| 指标 | 结果 |
| --- | --- |
| 到点率 | 0%（0/3） |
| 平均真实路径长度 | 4.309370 m |
| 超时率 | 100%（3/3） |
| 平均回合累计奖励 | -97.121903 |

| seed | 结果 | 控制步数 | 路径 m | 累计奖励 | 最近距离 m | 最终距离 m | 停滞步数 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 7 | timeout | 600 | 4.309370 | -97.121903 | 1.276728 | 2.460853 | 461 |
| 17 | timeout | 600 | 4.309370 | -97.121903 | 1.276728 | 2.460853 | 461 |
| 23 | timeout | 600 | 4.309370 | -97.121903 | 1.276728 | 2.460853 | 461 |

每回合覆盖完整 120 秒仿真时间；到点阈值为 0.35 m。路径按返回的 0.2 秒控制帧真值位置累计，回合奖励不折扣。完整定义及故障分类见 [评估说明](navigation-evaluation.md)。

## 解释与边界

三次重复结果一致。轨迹先接近目标，最近距离约 1.28 m，之后未达到目标并在后段持续停滞，最终距目标约 2.46 m。停滞来自环境已有判定：最近 2 秒位移不足 0.05 m，且平均指令线速度绝对值大于 0.05 m。本次没有接触传感器证据，因此不把停滞直接认定为碰撞。

当前地图、初态与初始前方 3 m 目标固定；seed 7、17、23 是同一场景的重复基线，不是三个不同任务。这组结果不估计跨场景成功率。被测模型只经历了先前 64+8 步短验收训练，本次只做确定性推理，没有梯度更新，也未加载或写入 Replay Buffer。

## 被测对象和复测证据

- 检查点：`step-72-a9b36d71`，schema `local-drive-v2`，profile `acceptance`。
- 模型 SHA256：`36fd1d011d8bc598e35e3016505d53028579004d8f996288efb2fb40ff7f2cd5`。
- CUDA 推理，仿真 Mesa llvmpipe EGL；冻结环境与后端源码与已验收离线包一致。
- 基础仓库提交：`3a8a3cefe60a43f0c5ffea56ee4c5179d0dd6ddb`。本次新增评估模块的实际源码和哈希随数据保存。
- 原有 20 项测试及新增 10 项评估测试全部通过。
- 独立从 JSONL 复算全部 1800 条转换的路径、奖励和汇总指标；回合/动作编号、ACK、200 个物理步及传感器时序全部通过。
- 检查点模型、回放、清单运行前后 SHA256 不变。旧训练权重继续使用 [此前已发布的数据包](acceptance-wsl2-rtx4060-sync-20261013.md)，本次不重复打包。

[完整结果](evidence/navigation-baseline-20261013/evaluation/evaluation.json) · [逐步轨迹与日志](evidence/navigation-baseline-20261013/) · [独立复算记录](evidence/navigation-baseline-20261013/trace-verification.json)

复现命令（在 `training/local_drive` 中，输出目录必须不存在）：

```bash
DOCKER_CONTEXT=wsl-native bash scripts/evaluate-vehicle.sh \
  /absolute/path/to/training/checkpoints/step-72-a9b36d71 \
  /absolute/path/to/new-evaluation-output --seeds 7 17 23 --max-steps 600
python3 scripts/verify_evaluation.py /absolute/path/to/new-evaluation-output/evaluation
```

下一步可用相同评估入口比较后续训练检查点，先检查同场景到点率和奖励是否改善；长期训练故障恢复、周期性自动评估和场景随机化仍是后续工作，本次没有启动正式长训练。
