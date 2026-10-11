# CPU、GPU与大文件分发

## 两种GPU用途必须分开

- **训练计算**：`--device cpu`、`--device cuda`、`--device auto`。默认cpu；auto按PyTorch实际检测选择；显式cuda不可用就报错，不能悄悄回退CPU。
- **仿真传感器渲染**：本次Gazebo gpu_lidar通过NVIDIA/EGL验收。训练选择cpu不会关闭仿真GPU渲染。

CPU镜像 `robot-trainer:local-drive-cpu-v2` 内含CPU版PyTorch，只能运行CPU；auto也会选择CPU。
CUDA镜像安装PyTorch2.8.0/cu126，可用同一CLI选择cpu或cuda；已在本机GTX 1650上通过短训练验收。完整使用方法见[CUDA训练器](cuda.md)，发布状态与digest见 `../provenance/ghcr-images.json`。

从仓库根目录进入 `training/local_drive`：

```bash
# 已验收CPU路径
docker build -f docker/Dockerfile.trainer -t robot-trainer:local-drive-cpu-v2 .
mkdir -p runs/cpu
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD/runs/cpu:/runs" \
  robot-trainer:local-drive-cpu-v2 --device cpu --steps 128

# 可选CUDA配方：首次使用必须在目标GPU上执行短测
docker build -f docker/Dockerfile.trainer.cuda -t robot-trainer:local-drive-cuda-v1 .
mkdir -p runs/cuda
docker run --rm --gpus all --user "$(id -u):$(id -g)" -v "$PWD/runs/cuda:/runs" \
  robot-trainer:local-drive-cuda-v1 --device cuda --steps 128
```

CUDA安装源来自[PyTorch官方2.8.0安装说明](https://pytorch.org/get-started/previous-versions/)。
CUDA新增依赖在镜像内 `/opt/robot-training/cuda-requirements.resolved.txt` 记录，完整版本清单已归档在 `../provenance/trainer-cuda-requirements.lock`。
不要将旧版CPU lock中的torch重新装回CUDA镜像。驱动及GPU架构仍需与所选CUDA wheel相容。
当前所有训练入口仍然是Pendulum工具链短测试，尚未实现真实Gazebo同步训练后端。

## 什么放Git，什么放附件

| 内容 | 位置 |
|---|---|
| Python代码、Dockerfile、配置、接口说明、小型验收JSON | 普通Git |
| 本次仿真镜像与CPU/CUDA训练器镜像 | GHCR，使用docker pull |
| 本次约1.5GB镜像tar与校验和 | 保留本地离线迁移备用 |
| 后续少量需要版本管理的模型权重 | 可选Git LFS；先确认存储/流量配额 |
| 高频checkpoint、Replay Buffer、长日志 | 本地/对象存储，不进普通Git |

GitHub普通Git拒收大于100MiB的单文件；Releases单个附件须小于2GiB，因此本次镜像包可直接作为一个Release附件，无需分卷。
来源：[Git大文件限制](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)、[Release限制](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)。

## 本次镜像包的版本边界

先前迁移包 `local-drive-images.tar` 是已经验收的CPU基线，来自独立本地源码提交 `644781c69eb541fc9953667b2d204580e2c3bc45`。
它含 `robot-sim:local-drive-20261011` 和 `robot-trainer:local-drive-20261011`，**不包含本次新增--device参数或CUDA配方**。
对应源码bundle、源码tar和SHA256SUMS一起保存。归档文件不被覆盖；本次新增设备选择代码位于当前目录。
当前CPU-v2新镜像单独测试，不冒充已归档旧镜像。`provenance/final-images.json`、`docs/evidence/`是初始基线记录。

本次Git提交不包含大镜像或任何训练权重。附件尚未上传时，不提供虚假的下载链接。

## GHCR分发

用户选择GHCR作为镜像分发渠道；不创建Release，不把tar放进Git。
镜像地址、digest及拉取命令见[GHCR说明](ghcr.md)。
Docker配置中已有GHCR登录，推送复用已有凭据；源码通过GitHub连接器提交。
如果另一个环境需要私有包登录，使用 `docker login ghcr.io -u huam54925-cpu`，在密码提示处输入有read:packages的PAT classic。
推送需要write:packages；不要把token写入代码、命令参数或聊天。
官方说明：[GHCR认证和权限](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)。
