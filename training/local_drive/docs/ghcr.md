# GHCR镜像分发

使用独立镜像发布仿真与训练器，普通Git只保存源代码、配置和小型验收证据。
本次目标Linux amd64；仿真需要NVIDIA Container Toolkit和兼容驱动；CPU训练器不要求GPU。

```bash
docker pull ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-sim:20261011
docker pull ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-trainer:20261011-cpu-v2
```

两份镜像均已推送并按digest拉取核验，RootFS层与已验收本地镜像一致。该核验使用本机现有层缓存，异机完整下载及运行仍待验收。
固定版本的digest记录在 `provenance/ghcr-images.json`，需要严格复现时使用 `镜像名@sha256:...`。
本次匿名访问两份镜像均被拒绝，需要使用有包读取权限的GitHub账号登录：`docker login ghcr.io -u YOUR_GITHUB_USERNAME`。
在密码提示处输入有read:packages权限的PAT classic；推送使用write:packages。不需要重新给Codex粘贴密钥。
发布镜像增加 `org.opencontainers.image.source` 标签，关联本GitHub仓库。

## 拉取后短验收

从主仓库执行 `cd training/local_drive` 后：

```bash
SIM_IMAGE=ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-sim:20261011 \
  bash scripts/accept-sim.sh
TRAINER_IMAGE=ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-trainer:20261011-cpu-v2 \
  python3 scripts/accept-trainer.py "$PWD/runs/ghcr-trainer"
```

直接运行CPU或自动选择：

```bash
mkdir -p runs/cpu
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD/runs/cpu:/runs" \
  ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-trainer:20261011-cpu-v2 \
  --device cpu --steps 128
```

该发布镜像是CPU版PyTorch，auto将选择CPU，显式cuda会报错。
CUDA构建配方见 `docker/Dockerfile.trainer.cuda`；本次没有发布未经验证的CUDA镜像。
无论选择什么计算设备，当前训练脚本都只是Pendulum工具链短测试，真实Gazebo同步训练后端仍待实现。

## 再次发布

本机已有两份已验收本地镜像时，运行 `bash scripts/publish-ghcr.sh`。
推送使用Docker已有登录，不读取凭据到代码或日志。
本地原1.5GB离线迁移tar继续保留作为CPU初始基线归档，本次不上传Release。

参考：[GitHub官方GHCR说明](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)。
