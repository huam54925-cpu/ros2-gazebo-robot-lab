# 2026-10-11 环境封装验收

工作目录 `/mnt/robot_disk/codex-builds/2026-10-11`，独立 ext4 数据盘 `/dev/sdb2`。
Docker 数据目录 `/mnt/robot_disk/docker-engine`。本地源 `/mnt/robot_disk/ros_sim` 保持原状。
GitHub 只读参考版本 `73b2c38e64aed874abab65559790d41a1b7bab97`，所用室内场景与远端 blob 一致。

## 已验证

- 仿真镜像含场景与入口；无源码挂载、无X11、无DISPLAY、非host网络启动。
- 实际 ROS 2 Lyrical / Gazebo 10.5.0，GTX 1650，驱动595.91.07；OGRE2日志确认EGL。
- 最终脚本连续两次冷启动验收通过。360束激光，其中283束有效有限值；真值与odom可读取。
- 最终首轮短动作真值位移约0.0991m，停车后0.3仿真秒检测窗口内漂移0m。
- 暂停状态通过stats.paused与0.35墙钟秒内仿真时钟不变共同验证。
- 两次固定场景冷启动稳定后的xyz初始位置差0m（门槛1e-4m）。不等于随机场景reset或动力学完全可复现。
- 11项契约/检查点单元测试通过，含旧样本、跨回合、动作编号、字段泄漏、动作形状、编码、保存中断及校验失败。

## 调试中实际遇到的问题

1. 首次诊断所有数据检查完成后，进程退出码139。修复：退出前显式取消全部订阅，并释放native publisher/Node。
2. 后续一次复测Python绑定的world control请求返回失败/超时，原因未完全确定；诊断改用Gazebo原生CLI控制，再用订阅时钟确认效果。
   这只用于短验收，不能替代未来常驻训练后端的控制/步进插件。
3. 宿主Python3.14没有ensurepip，通用venv不承担训练依赖；训练器独立使用Docker内Python3.11。

原始失败日志保存在构建目录 logs/sim-first 与 logs/sim-cold-repeat；最终通过证据在 docs/evidence/。

## 范围限制

真实GazeboBackend、精确action ACK/step、Gymnasium车辆适配器尚未实现；训练入口明确拒绝Gazebo模式。
没有车辆训练效果、通道通过率或脱困率结论，没有部署车辆权重。
没有contact传感器，因此不声称通过接触检测或无碰撞验收。
初始验收时异机/WSL2未验收；后续单GPU WSL2异机结果见[RTX 4060增量记录](acceptance-wsl2-rtx4060.md)。多GPU、多worker和跨容器训练通信仍未验收。

## 训练器短验收

Python3.11 / SB3 2.7.0 / PyTorch2.8.0+cpu / Gymnasium1.2.1，完整依赖版本在 provenance/。
Pendulum-v1 实跑128步、保存模型和Replay Buffer、重新加载后继续32步，共160步，Replay Buffer为160条。
保存/加载前确定性动作最大差为0；模型哈希和Replay Buffer哈希核验通过。
学习、保存、恢复及继续运行合计约5.53墙钟秒（不含容器启动和首次导入，不能当车辆采样吞吐）。
另一次真实SIGTERM在第29步停止，Replay Buffer保留28条（SB3回调在当前步入库前停止），成功保存和重新加载，进程退出码0。
这些是工具链测试，不代表Pendulum任务收敛，也不是车辆训练结果。

## 迁移包回读验收

完整镜像tar共 1,592,029,184 bytes（约 1.48 GiB），SHA256：
`8655b0d8da39ee0453c97745c6cc8b108e9f1c975ac05d9fec9d02d7d2da2491`。
两个镜像 `docker save` 导出、`docker load` 导入成功，导入前后镜像ID一致。
导入后的仿真再次通过同一完整短验收，证据位于 `docs/evidence/sim-after-load/`。
导入后的训练器又完成32+32步短测，Replay Buffer为64条，加载前后动作差为0。
这是同一Docker daemon的回读与重新运行验证；没有删除原镜像层，也没有在另一台机器实测。
因此可交付迁移包及复测脚本，但异机验证仍为待验收。

初始验收时，源码与小型验收证据保存在未配置远端的独立本地仓库。
后续按用户授权，将源码提交到主仓库的独立功能分支与PR #7，镜像通过GHCR分发；镜像与checkpoint不进入Git。


## CPU/CUDA设备选择增量

源代码新增 --device cpu/cuda/auto；CPU-v2容器14项单元测试通过，128+32步及真实SIGTERM保存测试通过，加载前后动作差0。
CUDA不可用且显式请求cuda时会报错，不会回退CPU；此阶段仅提供CUDA配方，后续GPU实测结果见下方增量记录。
GHCR发布状态与digest见ghcr.md；以上初始镜像tar的基线记录保留不变。

CLI额外验证：CPU镜像中 --device auto 选择cpu并完成32+32步；--device cuda 以退出码2明确拒绝不可用设备。

## GHCR发布验收

仿真与CPU-v2训练器均已推送GHCR，并成功按digest拉取。发布镜像只增加OCI标签；
两者RootFS层分别与已通过短验收的本地镜像完全一致。digest与拉取引用见 `../provenance/ghcr-images.json`。
匿名manifest访问两者均被拒绝，已登录访问成功；访问记录见 `../provenance/ghcr-access.json`。
本次拉取在已有镜像层缓存的同一主机完成，验证远端发布、认证与manifest可读取；不替代异机完整下载与运行验收。

## CUDA训练器实跑增量

在本机NVIDIA GeForce GTX 1650（4GB、compute capability 7.5）、驱动595.91.07上实测。
容器内PyTorch为2.8.0+cu126，CUDA运行库12.6；CUDA矩阵乘法和反向传播通过，设备为cuda:0。
14项单元测试通过；指定cuda完成Pendulum 128+32步，Replay Buffer为160条，保存恢复前后动作差0。
另一次真实SIGTERM在第18步停止，Replay Buffer为17条，成功保存、核验哈希和重新加载，退出码0。
版本清单见 `../provenance/trainer-cuda-requirements.lock`，结构化证据见 `evidence/cuda/`。
这些是GPU环境及工具链短测，不代表车辆训练效果、GPU加速比或异机运行结果。

设备模式补测：透传GPU时auto选择cuda；不透传GPU且显式cpu时选择cpu，两者均完成32+32步及恢复一致性检查，见 `evidence/cuda/device-modes.json`。

## WSL2 / RTX 4060 异机增量

另一台Windows + WSL2机器已通过无显示仿真、CPU与CUDA训练器短验收。CUDA矩阵乘法和反向传播、14项测试、Pendulum 128+32步与SIGTERM保存恢复通过。
仿真为Mesa llvmpipe软件EGL渲染；CPU/CUDA训练器按锁定依赖本地构建，CUDA镜像未上传GHCR。
详见[异机部署、证据与训练边界](acceptance-wsl2-rtx4060.md)。车辆同步后端与Gymnasium车辆环境仍未实现，没有车辆训练权重。
