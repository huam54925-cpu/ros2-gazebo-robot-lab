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

上述 GPU 基线阶段尚未验证 Ogre2 / EGL、无 NVIDIA 主机、SLAM 或 Nav2；后续雷达阶段的 Ogre2 结果见下文。

## 2026-10-02 2D 雷达、扫描显示与运动回归

采用用户上传的最终 `vehicle.world.sdf`、`bringup.launch.py` 和 `vehicle.rviz`。运行证据来自用户本机终端输出、运动确认及截图；发布环境没有重新启动仿真或重建镜像。

### 配置与本机证据

| 检查 | 配置或观察结果 |
| --- | --- |
| 传感器 | chassis 内的 gpu_lidar，名称 lidar_2d |
| 安装位置 | 相对 chassis 为 [0, 0, 0.4] m，旋转为零 |
| 配置扫描范围 | 360°，360 个水平采样点，单层 |
| 配置量程 / 更新率 | 0.1–12 m / 10 Hz（仿真时间） |
| Sensors 渲染 / GUI 渲染 | Ogre2 / OGRE |
| Gazebo→ROS 桥接 | /scan，sensor_msgs/msg/LaserScan ← gz.msgs.LaserScan |
| 实际扫描 header | stamp=2.5 s，frame_id=vehicle/lidar |
| 固定 TF | vehicle/chassis → vehicle/lidar，平移 [0, 0, 0.4]，单位四元数 |
| 持续接收频率 | ros2 topic hz 输出 3.836–4.488 Hz |
| 一帧有效测距 | 约 3.7022–4.9267 m；其余多个方向为 inf |
| RViz | 截图可见墙面扫描点；上传配置包含启用的 LaserScan 和 /scan |
| 保存的显示设置 | Best Effort / Volatile，Points，3 像素，Decay Time=0 |
| 测试墙 | 中心 [4, 0, 1] m，尺寸 [0.3, 6, 2] m，静态、含 visual/collision |

Gazebo 初次查询显示无发布者，随后 ROS 收到扫描消息且频率检查持续返回；不能把初次查询视为持续故障。TF 查询先等待发现，随后连续返回固定变换；固定 TF 的 time=0 是预期行为。频率命令被 timeout 结束时出现 wait-set/context 错误，退出前已经收到连续数据。

实际接收频率低于配置频率。本阶段没有测量实时因子、仿真时间扫描间隔、丢帧率或实时性能；不能据此宣称已经实测达到 10 Hz。360 点是上传模型的配置值，未提供独立消息长度统计。截图证明扫描点可见，没有提供 LaserScan Status 字段的单独文本记录。

### 加入雷达后的运动回归

| 检查 | 本次结果 |
| --- | --- |
| 前进位移 | 0.23539999983581378 m |
| 转向角度 | 0.38699999999650114 rad |
| 停止后线速度 / 角速度 | 0.0 / 0.0 |
| 里程计坐标 | vehicle/odom → vehicle/chassis |
| 关节反馈 | left_wheel_joint、right_wheel_joint、caster_wheel |
| 原有 TF 查询 | chassis、left_wheel、right_wheel、caster 均成功 |
| 运动脚本 | passed: true；用户确认小车运动 |

发布时检查 Python 语法、XML/YAML 可解析性，以及扫描话题、frame_id、安装位置和 RViz 配置的一致性。功能证据支持雷达接入和原运动回归通过；SLAM、Nav2 与 EGL 专项测试尚未执行。
