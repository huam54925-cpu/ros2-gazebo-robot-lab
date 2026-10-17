# RTX 4060 本机停止与恢复验收（2026-10-17）

结果：通过。Windows + WSL2 Ubuntu、RTX 4060 Laptop 上使用真实 Gazebo 和正式 SAC 配置，首次采样 16 步后通过 STOP 停止，新进程从 16 步恢复至 32 步，再通过 SIGTERM 停止。两次调度器退出码均为 0、状态均为 stopped，所属容器残留均为 0。

| 检查项 | 首次运行 | 新进程恢复 |
| --- | --- | --- |
| 初始步数 | 0 | 16 |
| 停止时有效步数 | 16 | 32 |
| Replay Buffer 条数 | 16 | 32 |
| 请求停止方式 | STOP 文件 | 调度器 SIGTERM |
| 模型和 Replay SHA256 | 通过 | 通过 |
| 退出后所属容器 | 0 | 0 |

两次使用相同运行 UUID、源码哈希、镜像 ID、种子和正式配置。恢复后复核首次检查点，其模型和 Replay 文件均未改变。没有 Configuration mismatch，没有训练片段失败或重试。

第二阶段训练容器退出码为 143：已发布 32 步检查点和完整训练报告后，在请求停止的退出阶段收到 SIGTERM。调度器按显式停止请求归类为 stopped，自身退出码为 0。这里不将容器 143 记作自然退出 0，也不将它计为训练失败；检查点及 Replay 均已独立复核。

## 范围与方法

- 对应仓库分支：`feature/long-train-offline-20261016`；执行前核对远端提交 `272a85a06146570473288455c5d429048c537f7f`。
- 实际运行源码为离线包提交 `c0ffb4d4ff3d268eaa456a89939f0cbcfc4045d3`；后续提交仅补充交付说明与证据。源码未修改。
- 每片段 16 步，`--hours 0.2 --chunk-steps 16 --eval-every 0 --max-failures 1`；0.2 小时是每次进程的上限，并非实际运行时长。
- 检查点发布到 16/32 步后请求停止。因此本次证明片段边界的正常停止与跨进程恢复，不证明任意物理步中断、断电或强制终止时的数据恢复。
- 正式配置 `learning_starts=5000`、Replay 容量 200000、seed 7。32 步仍在预热期，梯度更新为 0、actor 参数变化为 0，符合配置预期。
- 学习设备 CUDA；Gazebo 渲染为 Mesa llvmpipe EGL。本次没有进行导航质量评估，也没有启动三小时训练。

## 环境与制品

训练镜像：`local-drive-sync-trainer:20261016-longrun`，Docker config ID `sha256:0c5150c9837521b1a1a1bee2dc2af024b8621e4cbd629e91bcaefad777205cea`。

仿真镜像：`local-drive-sync-sim:20261012-v2`，Docker config ID `sha256:7fae8a467efcf70a9a8956da5150ddbc624f6e3ecc785f513d717760ecdf5ad1`。

运行前源码与镜像清单检查、RTX 4060 CUDA 矩阵反向传播均通过。依赖为 PyTorch 2.8.0+cu126、SB3 2.7.0、Gymnasium 1.2.1。Docker context 为 wsl-native，WSL VHD 位于 D:\WSL\Ubuntu。启动前 D 盘物理可用约 148 GiB；WSL 内显示的逻辑可用空间不能代表 Windows 物理剩余容量。

完整模型和 Replay 原件保留于：

`/home/user111/AI-Training/diagnostics/stop-resume-20261017/run/checkpoints/`

本目录提交报告、执行命令、运行身份、日志、各阶段验证结果、检查点 manifest 和验收脚本。二进制模型及 Replay 未上传 Git；其 SHA256 已记录在 manifest 中。Windows 证据副本位于 `F:\AI-Training\diagnostics\stop-resume-20261017`。

关键证据：`summary.json`、`phase-1-verification.json`、`phase-2-verification.json`、`run/events.jsonl`、`run/reports/`、`run/checkpoints/*/manifest.json`。`accept-stop-resume-20261017.py` 是本次外部验收脚本，硬编码路径用于记录本机执行方式，未改动发行版运行源码。

下一阶段可另行启动三小时训练；本次验收结果不代表长时间稳定性或到点率已经改善。
