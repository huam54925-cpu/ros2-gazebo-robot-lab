# 小车运动、TF 与 RViz 基础

本机项目目录：`/mnt/robot_disk/ros_sim`。2026-10-02 已在 GTX 1650 / NVIDIA 595.91.07 上验证硬件渲染：启动脚本同时使用 `--gpus all` 和 `--device /dev/dri:/dev/dri`，不再设置 `LIBGL_ALWAYS_SOFTWARE=1`。保留 Qt `threaded`、`xcb_glx`、OpenGL 和 Gazebo OGRE 设置。Gazebo 与 RViz 正常显示，运动与 TF 回归通过。

## 启动与停止

在宿主机终端运行：

```bash
/mnt/robot_disk/ros_sim/docker/start-navigation-base.sh
```

启动脚本在容器已存在时拒绝重复启动。停止当前仿真可在启动终端按 Ctrl+C，或执行 `docker stop robot-sim-gui`。容器会自动删除，项目文件和用户配置仍保留在移动硬盘。Gazebo 服务端与 GUI 是独立进程，关闭 GUI 不等于停止服务端。

脚本沿用镜像 `robot-sim:lyrical-v1`、现有显示授权文件 `gui/xauth`、挂载和用户身份；下次桌面登录后若出现 X11 授权错误，需要更新显示授权文件。

## 修复内容

- Gazebo 继续使用完整物理模型；ROS 描述副本去掉模型级 `<pose>`，解决 sdformat_urdf 的解析限制。原始模型和世界文件保留为 `*.original.sdf`。
- ROS 描述把球形后轮关节设为固定，仅用于机器人显示；Gazebo 中仍然是 ball 关节。球形后轮的旋转不会改变其外观。这不是后轮姿态的完整运动学表达。
- DiffDrive 是唯一里程计来源，30 Hz 仿真时间频率。去掉另一个 OdometryPublisher，避免同一话题混入两套坐标定义。
- 速度仅 ROS→Gazebo；里程计、时钟、关节、里程计 TF 仅 Gazebo→ROS。
- 所有 ROS 节点使用仿真时间。
- `robot_state_publisher` 仍有 KDL 根链接惯量警告；它不影响本次显示与 TF，模型已成功初始化。

## 坐标与显示

`vehicle/odom → vehicle/base_link → vehicle/chassis → {vehicle/left_wheel, vehicle/right_wheel, vehicle/caster, vehicle/lidar}`。

第一段由 DiffDrive 提供；base_link → chassis 由静态 TF 发布，平移为 [-0.7057095, 0, 0.5] m；车体关节由 robot_state_publisher 发布，雷达使用独立静态 TF。base_link 位于轮轴中心的地面投影，里程计以它的起始位姿为参考；不能直接作为 Gazebo 世界坐标。

RViz 配置：固定坐标 `vehicle/odom`，机器人描述话题 `/robot_description`（Transient Local），TF 前缀 `vehicle`；包含 RobotModel、TF、Grid、Odometry 和 LaserScan。LaserScan 订阅 `/scan`，采用 Best Effort / Volatile，Points 样式、3 像素、Decay Time=0。mapping 场景另用 slam/mapping.rviz，已接入 map 和 SLAM；Nav2 使用独立 navigation/navigation.rviz 配置和导航容器，见根目录 docs/nav2-navigation.md。

## 2D 雷达

`lidar_2d` 是挂载在 chassis 上的 `gpu_lidar`，局部位置 `0 0 0.4`、旋转为零。`bringup.launch.py` 中的固定 TF 与该位置一致，消息 frame_id 为 `vehicle/lidar`。它由独立的 static_transform_publisher 发布；ROS 描述中没有添加新的物理 link。

配置为 360°、360 个水平采样点、10 Hz 仿真时间更新率、0.1–12 m 量程。world 显式启用物理、用户命令、场景广播和传感器系统；Sensors 使用 Ogre2，Gazebo GUI 继续使用 OGRE。静态测试墙中心位于世界坐标 `4 0 1`，尺寸为 `0.3 6 2` m。

2026-10-02 用户本机验证收到 `/scan` 消息，固定 TF 查询成功，RViz 显示墙面的扫描点；实际接收频率约 3.8–4.5 Hz，有效回波约 3.70–4.93 m。实时因子未测量。

RViz 的 File → Save Config 会更新容器内 `/work/robot_ws/navigation_base/vehicle.rviz`，对应宿主机本项目的 `workspace/navigation_base/vehicle.rviz`。

## 复验运动和反馈

以下测试会短暂前进、转向，然后停止；应只针对这个仿真容器运行：

```bash
docker exec robot-sim-gui bash -lc 'source /opt/ros/lyrical/setup.bash; python3 /work/robot_ws/navigation_base/verify_motion.py'
```

检查 `/robot_description`、关节状态、里程计位移与转角、停止速度，以及从里程计到所有可视链接的 TF。测试在正常异常退出路径中都会发送停止命令。

2026-10-01 实测通过：前进约 0.275 m，转向约 0.416 rad，最终线速度/角速度均为 0。验证记录位于项目的 `logs/motion-verification.json`，启动记录为 `logs/navigation-base.log`。用户已确认 RViz 的机器人、TF 坐标轴和里程计箭头全部显示正常。

2026-10-02 硬件渲染回归再次通过：前进 0.3922 m，转向 0.6369 rad，最终线速度/角速度均为 0，`passed: true`。这次结果来自用户本机终端输出；完整阶段记录见 [验证记录](../../docs/validation.md)。

解析限制参考：https://github.com/ros/sdformat_urdf/tree/rolling/sdformat_urdf

加入雷达后运动回归通过：前进 0.23539999983581378 m，转向 0.38699999999650114 rad，停止速度 `[0.0, 0.0]`，`passed: true`。
