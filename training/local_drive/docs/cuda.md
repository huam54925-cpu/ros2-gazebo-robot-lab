# CUDA训练器

CUDA镜像与CPU镜像使用相同训练入口，可用 `--device cpu|cuda|auto` 选择计算设备。
镜像内自带PyTorch CUDA运行库；宿主机需要兼容的NVIDIA驱动及NVIDIA Container Toolkit。
`nvidia-smi` 显示的CUDA版本是驱动支持上限，不等同于容器内PyTorch运行库版本。
本配方使用PyTorch 2.8.0 / CUDA 12.6，并固定Python基础镜像digest。

## 拉取与运行

发布状态及不可变digest以 `../provenance/ghcr-images.json` 为准。
先使用有包读取权限的GitHub账号登录GHCR，然后：

```bash
docker pull ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-trainer:20261011-cu126-v1
# 从仓库根目录进入 training/local_drive
mkdir -p runs/cuda
docker run --rm --gpus all --user "$(id -u):$(id -g)" \
  -v "$PWD/runs/cuda:/runs" \
  ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-trainer:20261011-cu126-v1 \
  --device cuda --steps 128
```

输出目录建议放在非系统盘。使用GPU必须同时透传设备并选择cuda；训练CLI默认仍是cpu。
`--device auto` 按容器实际可见设备选择；显式cuda不可用时退出报错。
GPU训练计算与Gazebo的激光渲染是两个用途，显存由两者共同占用。
这里运行的仍是Pendulum工具链测试，不是车辆训练，也不能据此判断GPU训练加速比。

## 换机器后验收

完整短验收包含单元测试、128+32步保存恢复以及SIGTERM停止保存：

```bash
TRAINER_IMAGE=ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-trainer:20261011-cu126-v1 \
  python3 scripts/accept-trainer.py --device cuda --gpus "$PWD/runs/cuda-acceptance"
```

输出目录必须尚不存在。另可单独验证CUDA矩阵运算及反向传播：

```bash
docker run --rm --gpus all --entrypoint python \
  -v "$PWD/scripts/cuda_probe.py:/tmp/cuda_probe.py:ro" \
  ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-trainer:20261011-cu126-v1 \
  /tmp/cuda_probe.py
```

## 本地构建及发布

```bash
docker build -f docker/Dockerfile.trainer.cuda -t robot-trainer:local-drive-cuda-v1 .
TRAINER_IMAGE=robot-trainer:local-drive-cuda-v1 \
  python3 scripts/accept-trainer.py --device cuda --gpus "$PWD/runs/cuda-local-acceptance"
bash scripts/publish-ghcr.sh cuda
```

完整依赖解析结果保存在镜像内 `/opt/robot-training/cuda-requirements.resolved.txt`。
具体测试硬件、结果和限制见 [验收记录](acceptance.md)。

参考：[PyTorch官方版本安装说明](https://pytorch.org/get-started/previous-versions/)、[NVIDIA驱动与CUDA版本说明](https://docs.nvidia.com/datacenter/tesla/drivers/latest/cuda-toolkit-driver-and-architecture-matrix.html)。
