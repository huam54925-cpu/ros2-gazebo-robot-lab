# 验证记录

## 2026-10-01 软件渲染基线

在本地 Docker 镜像 `robot-sim:lyrical-v1` 上执行了运动与 TF 验证，结果：

| 检查 | 结果 |
| --- | --- |
| 前进里程计位移 | 0.2746 m |
| 转向角度 | 0.4161 rad |
| 停止后线速度 / 角速度 | 0 / 0 |
| 里程计坐标 | vehicle/odom → vehicle/chassis |
| 关节反馈 | left_wheel_joint、right_wheel_joint、caster_wheel |
| TF 查询 | odom 到车体、左右轮、后轮全部通过 |
| robot_description | 已收到 |
| 里程计发布者数 | 1 |
| RViz | 用户确认机器人、TF 坐标轴、里程计箭头显示正常 |

该次验证实际使用 Mesa llvmpipe 软件渲染；当时尚未验证 NVIDIA 加速、SLAM 或 Nav2。

发布整理时另行检查了 Shell/Python 语法、XML/YAML 可解析性、显示准备脚本和 Git 上传文件范围；未重新构建镜像，也未重新启动已暂停的仿真。

## 2026-10-02 NVIDIA 硬件渲染与运动回归

证据来自用户在本机运行后提供的终端输出及 GUI 显示反馈。仓库更新过程中只做静态检查，没有在编辑环境重新运行 Gazebo、RViz 或运动测试。

### 环境与启动修正

| 项目 | 本机结果 |
| --- | --- |
| GPU | NVIDIA GeForce GTX 1650，4 GiB |
| NVIDIA 驱动 | 595.91.07 |
| Docker 镜像 | `robot-sim:lyrical-v1` |
| ROS 2 / Gazebo | Lyrical / Gazebo Sim 10.5.0 |
| 容器设备 | `--gpus all` 与 `--device /dev/dri:/dev/dri` |
| Qt / Gazebo 渲染设置 | xcb、xcb_glx、OpenGL、threaded / OGRE |
| 强制软件渲染 | 已删除 `LIBGL_ALWAYS_SOFTWARE=1` |

此前容器没有 `/dev/dri`。加入设备映射后，同一镜像的 `glxinfo -B` 输出变为：

```text
direct rendering: Yes
OpenGL vendor string: NVIDIA Corporation
OpenGL renderer string: NVIDIA GeForce GTX 1650/PCIe/SSE2
OpenGL version string: 4.6.0 NVIDIA 595.91.07
```

宿主机 `nvidia-smi` 中出现 `gz-sim-gui-client` 图形进程。完整小车启动后，用户确认 Gazebo 与 RViz 正常显示；RViz 日志报告 `OpenGl version: 4.5 (GLSL 4.5)`。该 RViz 版本日志没有单独提供 renderer 厂商信息。

### 运动与反馈结果

运行：

```bash
docker exec robot-sim-gui bash -lc \
  'source /opt/ros/lyrical/setup.bash; python3 /work/robot_ws/navigation_base/verify_motion.py'
```

| 检查 | 本次结果 |
| --- | --- |
| 前进里程计位移 | 0.39219999983525 m |
| 转向角度 | 0.6368999999965219 rad |
| 停止后线速度 / 角速度 | 0.0 / 0.0 |
| 里程计坐标 | vehicle/odom → vehicle/chassis |
| 关节反馈 | left_wheel_joint、right_wheel_joint、caster_wheel |
| TF 查询 | odom 到 chassis、left_wheel、right_wheel、caster 全部返回 |
| 验证脚本 | `passed: true` |

验证脚本成功结束也意味着其等待的 `robot_description` 已收到。硬件渲染设置下，原有控制、里程计、关节反馈和 TF 验证继续通过。固定墙钟时间内的位移变化不构成渲染性能或实时性基准。

最初的 `gz sim -g` 测试只启动 GUI，因此反复等待 world 列表；完整启动文件分别启动 server/world 与 GUI 后正常连接。后续验收以完整小车启动结果为准。

本阶段尚未验证 Ogre2 / EGL、无 NVIDIA 主机、SLAM 或 Nav2。
