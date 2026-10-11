# Local Drive Training · 2026-10-11

独立训练封装目录。从主仓库根目录先执行 `cd training/local_drive`。复用 Robot Exploration 2.0 的现有场景与 ROS/Gazebo 基础镜像。
目标是环境迁移、算法设计与短验收；今天不声称获得车辆训练效果。

**最新异机进展：** WSL2 / RTX 4060 已通过仿真及CPU/CUDA训练器验收，见[部署记录与训练边界](docs/acceptance-wsl2-rtx4060.md)。当前能运行Pendulum训练工具链；小车训练仍需实现同步Gazebo后端与Gymnasium车辆环境，没有车辆权重。
原 `/mnt/robot_disk/ros_sim` 工作区保持不变。本目录将初始封装导入GitHub；原仿真入口保留。

## 内容

- `docker/Dockerfile.sim-train`：ROS 2 Lyrical / Gazebo 10，无显示、自带场景，默认暂停，不启动SLAM。
- `docker/Dockerfile.trainer`：独立 Python 3.11 / CPU PyTorch / SB3 容器。
- `training/configs/sac_local.json`：SAC 局部驾驶设计参数。
- `training/envs/contracts.py`：接口协议、动作映射、激光编码、时序检查。
- `training/checkpoints.py`：模型与 Replay Buffer 原子发布、SHA256核验。
- `training/train.py`：Pendulum 小环境短训练/恢复工具链验收。
- `docs/algorithm.md`：SAC任务、奖励、回放更新、实现顺序和评价方案。
- `docs/interface.md`：观测295维、奖励、reset/step、停车、故障与部署约定。
- `docs/acceptance.md`：本次真实验收结果及尚未实现的部分。
- `provenance/`：选取的本地工作区文件、来源版本、镜像信息及依赖记录。

## CPU/GPU与发布

训练器新增 `--device cpu|cuda|auto`，默认CPU。CPU-v2和CUDA版均已做短测；CUDA版在本机GTX 1650上通过运算、短训练与保存恢复验收，见[GPU使用说明](docs/cuda.md)。
GPU完整镜像上传已停止；另一台机器使用同一源码，按[GPU依赖配置](docs/cuda.md)安装即可。CPU镜像不会仅因透传GPU而自动变成CUDA版。
仿真的GPU激光渲染与训练计算设备独立。详见[设备选择与大文件分发](docs/devices-and-artifacts.md)。
先前1.5GB迁移包为原CPU基线，匹配其附带源码，不包含新增device参数。

## 从GHCR直接启动

已验收镜像通过GHCR分发，见[拉取命令与验收步骤](docs/ghcr.md)。无需把1.5GB的离线tar提交到Git。

## 构建和短测试

需要 Linux amd64 Docker。仿真使用 NVIDIA GPU 与 NVIDIA Container Toolkit；训练器的 CPU 镜像不需要 GPU。
基础镜像 `robot-sim:lyrical-v1` 来自本机已有封装，未发布到公共 registry。
无此基础镜像时可直接拉取GHCR上的完整仿真镜像，或导入本次离线迁移包；不能只复制 Dockerfile 就声称可离线重建。

```bash
bash scripts/build.sh
bash scripts/accept-sim.sh
mkdir -p runs/trainer
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/runs/trainer:/runs" robot-trainer:local-drive-cpu-v2 --steps 128
docker run --rm --entrypoint python robot-trainer:local-drive-cpu-v2 \
  -m unittest discover -s tests -v
```

`accept-sim.sh` 仅创建/删除它自己的诊断容器；默认没有任何源码或显示挂载。
默认输出 `runs/` 不进 Git；可传入非系统盘上的其他输出路径。
训练器命令只运行 Pendulum-v1，不支持车辆训练；`--backend gazebo` 会解释原因并退出。
本次示例检查点只证明保存/加载一致性，不能部署到车上。

完整短验收（含SIGTERM停止保存），输出目录必须尚不存在：

```bash
python3 scripts/accept-trainer.py "$PWD/runs/trainer-acceptance"
```

依赖最终版本固定在 `provenance/trainer-requirements.lock`；构建脚本先核对本地基础镜像ID，再构建两个镜像。

## 迁移

```bash
# 导出完整镜像（在非系统盘）
bash scripts/export-images.sh /path/on/data-disk/migration
# 另一台同架构、满足GPU驱动条件的机器
cd /path/to/migration
sha256sum -c SHA256SUMS
docker load -i local-drive-images.tar
# 进入本项目源代码，再执行上述短测试
```

镜像内没有原机器源码路径依赖；挂载仅用于训练输出持久化。
迁移验收分为“本机save/load后重测”和“另一台机器重测”，报告分别记录，不混同。
没有远端机器时不宣称完成异机验收。镜像ID、依赖清单和源码哈希保存在 provenance/。

## 继续开发

先实现 Gazebo 内部动作确认与精确步进、冷reset和Gymnasium适配器，然后接入真实 SAC。
先做固定场景单worker验收，再做跨容器RPC、多场景课程和多环境隔离。
详见接口说明；原场景无接触传感器，接触奖励暂不启用。
