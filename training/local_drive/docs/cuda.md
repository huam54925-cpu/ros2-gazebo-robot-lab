# CUDA训练器

CUDA镜像与CPU镜像使用相同训练入口，可用 `--device cpu|cuda|auto` 选择计算设备。
镜像内自带PyTorch CUDA运行库；宿主机需要兼容的NVIDIA驱动及NVIDIA Container Toolkit。
`nvidia-smi` 显示的CUDA版本是驱动支持上限，不等同于容器内PyTorch运行库版本。
本配方使用PyTorch 2.8.0 / CUDA 12.6，并固定Python基础镜像digest。

## 本地构建与运行

本机GPU短测已通过。用户说明可以自行配置CUDA后，完整GPU镜像上传已停止；该GPU标签尚未完成发布，不能作为可拉取成品使用。
使用仓库中的构建配方和依赖清单即可重建；本机已有下方local镜像。

```bash
# 从仓库根目录进入 training/local_drive
docker build -f docker/Dockerfile.trainer.cuda -t robot-trainer:local-drive-cuda-v1 .
mkdir -p runs/cuda
docker run --rm --gpus all --user "$(id -u):$(id -g)" \
  -v "$PWD/runs/cuda:/runs" \
  robot-trainer:local-drive-cuda-v1 \
  --device cuda --steps 128
```

输出目录建议放在非系统盘。使用GPU必须同时透传设备并选择cuda；训练CLI默认仍是cpu。
`--device auto` 按容器实际可见设备选择；显式cuda不可用时退出报错。
GPU训练计算与Gazebo的激光渲染是两个用途，显存由两者共同占用。
这里运行的仍是Pendulum工具链测试，不是车辆训练，也不能据此判断GPU训练加速比。

## 换机器后验收

完整短验收包含单元测试、128+32步保存恢复以及SIGTERM停止保存：

```bash
TRAINER_IMAGE=robot-trainer:local-drive-cuda-v1 \
  python3 scripts/accept-trainer.py --device cuda --gpus "$PWD/runs/cuda-acceptance"
```

输出目录必须尚不存在。另可单独验证CUDA矩阵运算及反向传播：

```bash
docker run --rm --gpus all --entrypoint python \
  -v "$PWD/scripts/cuda_probe.py:/tmp/cuda_probe.py:ro" \
  robot-trainer:local-drive-cuda-v1 \
  /tmp/cuda_probe.py
```

## 后续需要完整镜像分发时

```bash
docker build -f docker/Dockerfile.trainer.cuda -t robot-trainer:local-drive-cuda-v1 .
TRAINER_IMAGE=robot-trainer:local-drive-cuda-v1 \
  python3 scripts/accept-trainer.py --device cuda --gpus "$PWD/runs/cuda-local-acceptance"
bash scripts/publish-ghcr.sh cuda
```

若直接在自己的Python环境安装，安装来源和版本见Dockerfile，完整版本清单见 `../provenance/trainer-cuda-requirements.lock`；PyTorch使用cu126官方索引。
完整依赖解析结果保存在镜像内 `/opt/robot-training/cuda-requirements.resolved.txt`。
具体测试硬件、结果和限制见 [验收记录](acceptance.md)。

参考：[PyTorch官方版本安装说明](https://pytorch.org/get-started/previous-versions/)、[NVIDIA驱动与CUDA版本说明](https://docs.nvidia.com/datacenter/tesla/drivers/latest/cuda-toolkit-driver-and-architecture-matrix.html)。
