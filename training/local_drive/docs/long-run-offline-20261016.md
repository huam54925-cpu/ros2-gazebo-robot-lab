# 三小时局部驾驶训练：离线部署与停止恢复

本次目标是固定 `indoor_fixed` 场景、给定前方3米目标的SAC局部驾驶训练，不是未知地图自主探索。模型、295维观测、奖励、0.2秒同步协议保持原样。允许倒车不等于已学会倒车；短训练产生梯度不代表到点率改善。

## 目标机和不需要安装的组件

用户确认目标仍为历史验收的 Windows + WSL2 Ubuntu 24.04、RTX 4060 Laptop 8GB、Docker context `wsl-native`。历史记录为驱动566.24、Docker29.1.3、NVIDIA Container Toolkit1.20.1；这不是本次远程实时探测，本次没有连接目标机。

复用已有驱动、WSL、Docker和GPU透传。无需在宿主机安装ROS、Gazebo、PyTorch、SB3、Gymnasium、CUDA Toolkit或Python虚拟环境；这些依赖已在镜像里。宿主仅需已有Python3、Docker CLI、bash、tar、gzip、sha256sum。

训练镜像 `local-drive-sync-trainer:20261016-longrun` 在已验收的 `20261012-v2` 上仅覆盖源码与测试，没有apt/pip安装。仿真镜像继续使用 `local-drive-sync-sim:20261012-v2`。CUDA负责学习，仿真仍使用Mesa llvmpipe EGL，不要求另装NVIDIA图形渲染栈。镜像仅导出本地文件，不上传Git或镜像仓库。

## 先确认不占Windows系统盘

把整个离线包拷到数据盘。WSL中的 `~/...` 不保证在Windows数据盘，根文件系统对应的VHD可能仍在C盘。先在PowerShell只读查看：

```powershell
Get-ItemProperty 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss\*' |
  Select-Object DistributionName, BasePath
```

确认目标WSL发行版的VHD位于数据盘。再在WSL中检查：

```bash
export DOCKER_CONTEXT=wsl-native
unset DOCKER_HOST
docker info --format '{{.DockerRootDir}}'
findmnt -T /你选择的训练数据目录
cat /etc/containerd/config.toml | head -20
df -h /你选择的训练数据目录
```

Docker Engine 29使用containerd镜像存储时，仅修改Docker的data-root不够；containerd的root也必须在正确磁盘，或整个WSL VHD位于数据盘。若VHD或Docker存储仍在系统盘，先完成存储迁移再导入；本包不自动迁移发行版、不改驱动、不修改daemon配置。优先使用数据盘上的ext4/WSL VHD，不把活跃训练结果写进 `/mnt/c` 或Windows NTFS目录。

建议导入镜像及训练结果预留至少60GB（另加已有模型空间）。完整200000容量Replay Buffer序列化可能很大，所有历史检查点会保留；调度器默认剩余空间低于10GiB就停止开启新片段，不自动删除模型。

## 导入与检查

在离线包根目录运行：

```bash
export DOCKER_CONTEXT=wsl-native
unset DOCKER_HOST
bash prepare_longrun_offline.sh
cd source/training/local_drive
export TRAINER_IMAGE=local-drive-sync-trainer:20261016-longrun
export LOCAL_DRIVE_SIM_IMAGE=local-drive-sync-sim:20261012-v2
export RUN_DIR=/数据盘路径/runs/sac-local-001
mkdir -p "$RUN_DIR"
python3 scripts/preflight_longrun.py --data-dir "$RUN_DIR"
```

导入脚本先检查SHA256，已有两张完全相同ID的镜像就跳过导入。不会下载或安装包。预检确认宿主源码与镜像源码一致，执行CUDA矩阵乘法与反向传播，输出GPU/依赖版本及Docker存储位置。预检不证明Gazebo训练已经验收。

镜像源码哈希匹配是严格门禁。不要在部署目录随意编辑受清单追踪的脚本后继续同一个实验；需修改时构建新镜像并新建运行目录。

## 先做短验收

```bash
python3 scripts/long_train.py --run-dir "$RUN_DIR" --hours 0.2 \
  --chunk-steps 500 --eval-every 0
```

另开终端，可随时请求停止：

```bash
touch "$RUN_DIR/STOP"
cat "$RUN_DIR/long-run-state.json"
```

必须等状态为 `stopped` 且进程退出后再恢复。若为 `failed`，先看events.jsonl和logs；不能将失败当正常停止。

```bash
rm "$RUN_DIR/STOP"
python3 scripts/long_train.py --run-dir "$RUN_DIR" --hours 0.05 \
  --chunk-steps 100 --eval-every 0
```

确认 `valid_steps` 没有清零，模型与Replay Buffer哈希有效。前5000步是正式预热，没有梯度更新是预期；不要为了看到梯度而把正式配置改成验收配置。

需要单独验证训练器确实能更新参数时，在新建的独立验收目录使用 `--profile acceptance` 的既有入口；该配置不能当正式训练或接入本调度器。

## 三小时正式运行

```bash
nohup python3 -u scripts/long_train.py --run-dir "$RUN_DIR" --hours 3 \
  --chunk-steps 1000 --eval-every 5000 --eval-seed 7 \
  > "$RUN_DIR/manager-console.log" 2>&1 < /dev/null &
```

`--hours 3`是本次调度墙钟预算，不含启动预检，不是净GPU训练三小时。预算包含容器启动、保存和评估；到达预算后请求保存退出，再执行有界清理，因此退出时间可能额外增加几分钟。单个片段默认最长1800秒（`--chunk-timeout`可调），正常返回但无新增步数也计失败；最多重试3次。不是永久自动续跑。

每片段从已保存模型、优化器和Replay Buffer恢复；Gazebo从新回合开始，不保证恢复随机数和物理场景的逐位轨迹。5000步评估使用固定seed7的一次回合，是趋势观察，不是多地图泛化成功率。需要更完整评估时使用仓库独立评估入口。

## 停止、异常和输出

- `STOP`、SIGTERM、SIGINT在训练/评估阶段均归为正常停止，部分评估不作为完整质量结果。
- Docker CLI有超时；daemon失联且不能确认清理时返回失败，禁止盲目继续新片段。
- 启动前持久化运行UUID和片段编号；训练、评估及其仿真容器都带该UUID标签。异常后恢复先清理属于这个运行的遗留容器，不操作其他项目容器。
- 同目录运行由文件锁排他。不要同时从另一台主机共享同一输出目录。
- 恢复时镜像ID、源码、种子和上下文不同会拒绝；请保留本次镜像标签与离线包。
- run-identity.json记录镜像和源码；long-run-state.json记录阶段；events.jsonl追加事件；logs与reports按片段编号保存；checkpoints/latest.txt指向完整模型；evaluations保存逐步轨迹和评估摘要。
- 主机断电、磁盘故障或强制终止不能保证最后未发布片段不丢失；本调度器不会自动开机或改变云平台资源。

## 本轮修改与原评估说明

修复真实跨进程续跑时SB3修改policy_kwargs、污染检查点配置而触发Configuration mismatch的问题：构造模型使用配置深拷贝，检查点保留原始配置。策略参数不变。旧版本已污染的检查点不自动改写，需保留原文件并单独迁移验证。

修复下载版long_train.py的无CLI超时、启动后日志失败漏清理、孤儿训练容器恢复和评估STOP误报失败问题；新增按运行UUID标记仿真子容器。未改同步动作/回执验证、策略结构、奖励或任务。

原评估中的固定场景范围、缺少空间记忆和跨地图证据等判断与代码一致。CUDA可用不代表NVIDIA图形渲染成功；固定场景短测不代表完整自主探索。此次构建和实际验收发生在本机GTX1650，目标RTX4060需要按上面步骤再验收。

## 本机实测记录（2026-10-16）

构建机为GTX1650 4GB，数据盘为 `/mnt/robot_disk`。Docker data-root为 `/mnt/robot_disk/docker-engine`，containerd root为 `/mnt/robot_disk/containerd`。本次未安装宿主软件，镜像构建关闭网络并复用现有依赖。

- 宿主调度器回归测试12项、最终镜像内完整测试43项通过。
- CUDA矩阵反向传播通过；镜像为PyTorch2.8.0+cu126、SB3 2.7.0、Gymnasium1.2.1。
- 首次真实Gazebo验收在初始化 `/local_drive/command` 响应超时，尚未训练。已清理该回合；未重发不确定动作。
- 全新回合复测通过：采样64步、梯度更新48次，actor最大参数变化0.00788864；保存并重载后续跑8步，Replay Buffer由64增至72。

首次启动失败说明短测不能证明每次冷启动可靠；调度器只在确认旧容器清理后从检查点重开回合，达到失败次数上限则退出。第一次正式续跑还暴露了Configuration mismatch；修复前失败日志也保留，修复后重新建立独立正式测试目录。未做三小时稳定性实测，也未在目标RTX4060上远程执行。原始日志、镜像ID和源码哈希见 `docs/evidence/longrun-20261016/`。镜像清单中的base_revision是覆盖源码之前的Git基线；最终源码以source_hashes逐文件校验为准。

最终镜像的真实跨进程生命周期测试通过：正式配置从0到8步后写STOP，状态stopped；新进程从8步恢复到16步后发SIGTERM，状态stopped。两次退出后本运行标签的容器均为0。正式预热阶段没有梯度更新是预期，不把这16步当作学会驾驶的证据。

最终镜像也重新通过真实梯度验收：64步采样、48次梯度更新，重载后续跑8步，Replay Buffer共72条；actor最大参数变化0.00788864。
