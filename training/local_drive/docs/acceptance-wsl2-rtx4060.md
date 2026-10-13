# WSL2 / RTX 4060 异机验收（2026-10-11）

> 本页为历史 Pendulum 工具链记录。2026-10-13 已完成真实车辆同步后端与 CUDA 短训练异机验收，见 [后续报告](acceptance-wsl2-rtx4060-sync-20261013.md)；下文“待实现”描述只代表当时状态。

## 结论与边界

另一台 Windows + WSL2 机器已完成无显示小车仿真短验收，以及 CPU / CUDA 训练器本地部署与完整短验收。
这证明本机可以在 CPU 和 GPU 上执行 Pendulum 的 SAC 更新、保存模型和 Replay Buffer、恢复后继续运行，并在 SIGTERM 时保存退出。
**这不代表已经能训练小车，也不证明 Pendulum 已收敛、车辆任务已学会或 GPU 有加速收益。**

车辆任务、295 维观测、动作与奖励设计已有；还需实现以下链路：

1. Gazebo 内部动作确认（action ACK）。
2. 固定仿真步进与新激光、位姿、时钟同步，拒绝旧样本和跨回合样本。
3. 干净 reset，清理残余速度与旧观测。
4. Gymnasium 车辆环境的 reset/step、奖励与终止/截断逻辑接入 SAC。
5. 固定场景单 worker 端到端验收，之后才能正式训练、评估成功率并产出车辆权重。

现有 `sim_probe.py` 是异步诊断脚本，不能直接当训练 step；`--backend gazebo` 仍明确拒绝启动。
目前没有车辆权重。本次只增加部署验收证据，未实现车辆后端。

## 测试机器与版本

- Windows + WSL2，Ubuntu 24.04.2；WSL 内独立 Docker Engine 29.1.3，context 为 `wsl-native`。
- NVIDIA GeForce RTX 4060 Laptop GPU，8188 MiB，compute capability 8.9，驱动 566.24。
- NVIDIA Container Toolkit 1.20.1；容器 GPU 透传可用。
- Python 3.11.16、PyTorch 2.8.0+cu126、CUDA 运行库 12.6、SB3 2.7.0、Gymnasium 1.2.1。
- 被测训练源码：`dbe2900f5c3e6be3082d3f4b2032fbd0a7433ee0`。

## 验收结果

| 验收项 | 实际结果 |
| --- | --- |
| 无显示 Gazebo 仿真 | PASS；360 束激光，初始 283 束有效，真值和里程计可读取 |
| 前进、停车、暂停 | 前进 0.0990712743 m；停车窗口内漂移 0 m；最终暂停通过 |
| 仿真渲染 | EGL，Mesa llvmpipe 软件渲染；未证明 NVIDIA 图形渲染可用 |
| CPU 训练器 | 14 项测试、128+32 步、Replay Buffer 160 条、恢复动作差 0、SIGTERM 保存通过 |
| CUDA 运算 | cuda:0 上矩阵乘法和反向传播通过 |
| CUDA 训练器 | 14 项测试、128+32 步、Replay Buffer 160 条、恢复动作差 0、SIGTERM 保存通过 |

CUDA SIGTERM 在第 63 步停止，Replay Buffer 为 62 条；训练器重载并校验检查点成功，退出码为 0。
SB3 的停止回调发生在当前步入库前，因此停止步数与 Replay Buffer 条数可能相差 1。
结构化报告和测试日志见 [evidence/wsl2-rtx4060/](evidence/wsl2-rtx4060/)。

## 本地部署与来源

本机已加载仿真镜像 `ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-sim:20261011`，镜像 ID 为
`sha256:50fa99414f4588813d872ae98cd379cdbbe9e0bbfa39b160eb718c2f9d8b820e`。

CPU 镜像下载遇到 HTTP 401，因此本机按被测源码和锁定依赖构建
`robot-trainer:local-drive-cpu-v2`，并非对 GHCR CPU 成品镜像的异机拉取验收。
本机 CUDA 镜像为 `robot-trainer:local-drive-cuda-v1`，镜像 ID 见
[cuda-deployment.json](evidence/wsl2-rtx4060/cuda-deployment.json)。CUDA 镜像未上传 GHCR。

为避开代理长连接停滞，依赖下载到本地 wheel 目录，再以 `--network=none` 离线安装构建。
CUDA 新下载的包按官方 PyPI / PyTorch 索引提供的 SHA256 校验；全部 wheel 的本地 SHA256、离线 Dockerfile 和最终依赖清单随证据保存。
构建后 `pip check` 通过，最终安装版本与 `provenance/trainer-cuda-requirements.lock` 完全一致。
离线 Dockerfile 的 `wheels/` 来自该锁文件，`training/` 和 `tests/` 来自上述源码；大文件不提交 Git。

在本机 WSL 中，从仓库 `training/local_drive` 目录可重新验收（每次使用不存在的输出目录）：

```bash
DOCKER_CONTEXT=wsl-native \
TRAINER_IMAGE=robot-trainer:local-drive-cuda-v1 \
python3 scripts/accept-trainer.py --device cuda --gpus \
  "$PWD/runs/cuda-accept-$(date -u +%Y%m%dT%H%M%SZ)"
```

本次补齐的是这台 WSL2 机器的单 GPU 环境与工具链验证。多 GPU、多 worker、跨容器训练通信、车辆同步采样和真实车辆策略仍未验收。
