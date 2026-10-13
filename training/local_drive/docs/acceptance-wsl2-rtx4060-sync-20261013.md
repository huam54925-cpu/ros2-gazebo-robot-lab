# RTX 4060 车辆同步离线镜像复测 · 2026-10-13

离线 v2 仿真与训练镜像已在目标 WSL2 / RTX 4060 机器上执行原版 `run-offline.sh`，整套门禁通过。此次运行的是 `LocalDriveEnv` 车辆环境与 CUDA SAC，不再仅是此前的 Pendulum 工具链。短验收证明同步采样、梯度更新和保存恢复正常，不证明导航效果或训练收敛。

## 版本和运行条件

- 被测代码：[`897ace68dc66fc2addbcac4ef04cd2985df219fa`](https://github.com/huam54925-cpu/ros2-gazebo-robot-lab/commit/897ace68dc66fc2addbcac4ef04cd2985df219fa)，分支 `feature/local-drive-training-20261011`。
- 镜像：`local-drive-sync-sim:20261012-v2`、`local-drive-sync-trainer:20261012-v2`，由手机物理传输的 `local-drive-offline` 包导入；10 项包内 SHA256 均通过。
- 31 个运行源码文件与仓库一致（比较前统一 LF 换行）；未修改镜像内运行代码，未挂载开发源码。
- Windows + WSL2 Ubuntu；内核 `5.15.167.4-microsoft-standard-WSL2`；Docker Engine 29.1.3，`wsl-native` context。
- NVIDIA RTX 4060 Laptop 8188 MiB，驱动 566.24。训练使用 CUDA，仿真使用 Mesa llvmpipe EGL。
- 检查点记录 Python 3.11.16、PyTorch 2.8.0+cu126、SB3 2.7.0、Gymnasium 1.2.1。
- 初次 WSL 启动曾超时；关闭并重启 WSL 后恢复。随后原版离线脚本从校验到训练恢复完整通过。

包内 `images.json` 记录 OCI 索引摘要，本机 overlay2 导入后显示配置摘要。已从原始归档核验“索引 → linux/amd64 清单 → config”关系，两种 ID 对应同一镜像，见 [映射校验](evidence/wsl2-rtx4060-sync-20261013/image-identity-check-20261013.json)。

## 本次实际结果

| 项目 | 结果 |
| --- | --- |
| 单元与原生传输 | 20 + 4 项通过 |
| 冷重置与同步 | 三次冷 reset、12 个动作重复性通过；每步 200 × 1 ms，传感器时间容差 1e-6 秒 |
| 环境与随机审计 | SB3 / Gymnasium 检查通过；三回合共 36 条随机转换通过 |
| 故障门禁 | 缺失 ACK、过期激光、跨回合数据被拒绝；回放分别保持 2→2、4→4、6→6；清理与重建通过 |
| CUDA SAC | 初始训练 64 步，48 次梯度更新，actor 最大参数变化 0.007888644933700562 |
| 重载续训 | 模型预测及回放重载检查通过，继续 8 步，最终回放 72 条 |
| 检查点 | 周期 16/32/48/64 步，加训练结束 64 步和恢复后 72 步，共 6 个；12 个模型/回放文件 SHA256 全部通过 |
| 清理 | 测试结束无剩余容器 |

## 报告、数据与完整检查点

[证据目录](evidence/wsl2-rtx4060-sync-20261013/)包含汇总、原始同步帧、随机动作/295 维观测/奖励、故障报告、训练报告、原始测试日志、校验记录以及每个检查点的清单。日志保存为 `.txt`，内容保持原样。

- [完整结构化汇总](evidence/wsl2-rtx4060-sync-20261013/summary.json)
- [同步帧记录](evidence/wsl2-rtx4060-sync-20261013/acceptance/backend-traces.json)
- [随机转换审计](evidence/wsl2-rtx4060-sync-20261013/acceptance/random-audit.json)
- [训练结果](evidence/wsl2-rtx4060-sync-20261013/vehicle-training.json)
- [完整运行数据与全部模型/回放下载](evidence/wsl2-rtx4060-sync-20261013/wsl2-rtx4060-sync-20261013-full-results.zip)

证据目录中的 `wsl2-rtx4060-sync-20261013-full-results.zip` 包含原始结果目录及全部 6 个检查点，大小 10,158,889 字节，SHA256：

```text
8098b918829c48b04be3111f1cd4772f36ac4b82df925b524e7c64b3b20e74b5
```

解压后 `acceptance-20261013/training/checkpoints/latest.txt` 指向 `step-72-a9b36d71`。各检查点 `manifest.json` 分别记录 `model.zip` 与 `replay.pkl` 的 SHA256。它们是短验收权重，不能视为可部署导航策略。原始报告中的 `/results/...` 为容器内路径，Windows/WSL 路径记录的是本次执行位置。

复测需已有这两个离线镜像及包文件，在 Linux 文件系统中的包目录运行；输出目录必须不存在：

```bash
DOCKER_CONTEXT=wsl-native bash run-offline.sh /absolute/path/to/new-results
```

## 接下来可以做什么

1. **固定场景独立评估。** 实现确定性策略评估入口，记录到点率、路径长度、超时率和回合奖励，先为当前短验收模型建立基线。当前仅有固定室内场景与前方 3 m 目标；不同 seed 不能当作不同场景泛化证据。
2. **长训练故障恢复。** 在学习器采样边界丢弃失败转换，再停车、销毁并重建新回合，设置连续失败上限。当前 `acceptance/formal` 都会在基础设施异常时退出；不能直接把后端 reset 伪装成成功 step，也不重发动作或物理推进。
3. **较长训练与周期评估。** 正式配置 `learning_starts=5000`，应运行超过预热期的有界训练，并保存周期模型与评价指标，再判断增加训练时长是否有价值。本次 `learning_starts=16` 只为短验收，不作为正式训练建议。
4. **场景随机化和性能优化。** 有稳定评估后再扩展目标、初态与地图，测量冷 reset、软件渲染和采样耗时，决定是否值得做 GPU 图形渲染、多 worker 或并行环境。

以上为下一阶段工作建议，本次上传不声称这些能力已经实现。
