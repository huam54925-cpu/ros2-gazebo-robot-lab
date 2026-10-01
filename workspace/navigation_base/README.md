# 小车运动、TF 与 RViz 基础

项目目录：`/mnt/robot_disk/ros_sim`。暂停 NVIDIA 排查，使用明确的软件渲染设置：`LIBGL_ALWAYS_SOFTWARE=1`、Qt `threaded`、OpenGL、Gazebo OGRE。

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

`vehicle/odom → vehicle/chassis → {vehicle/left_wheel, vehicle/right_wheel, vehicle/caster}`。

第一段由 DiffDrive 提供，其余由 robot_state_publisher 提供；不再桥接 Gazebo 全部 link pose，避免对同一 TF 重复发布。DiffDrive 的平面里程计以起始车体为参考，z=0；它不代表底盘距离地面的真实高度，也不是地图坐标。

RViz 配置：固定坐标 `vehicle/odom`，机器人描述话题 `/robot_description`（Transient Local），TF 前缀 `vehicle`；包含 RobotModel、TF、Grid 和 Odometry。当前尚未添加 `map`、激光雷达、SLAM 或 Nav2。

## 复验运动和反馈

以下测试会短暂前进、转向，然后停止；应只针对这个仿真容器运行：

```bash
docker exec robot-sim-gui bash -lc 'source /opt/ros/lyrical/setup.bash; python3 /work/robot_ws/navigation_base/verify_motion.py'
```

检查 `/robot_description`、关节状态、里程计位移与转角、停止速度，以及从里程计到所有可视链接的 TF。测试在正常异常退出路径中都会发送停止命令。

2026-10-01 实测通过：前进约 0.275 m，转向约 0.416 rad，最终线速度/角速度均为 0。验证记录位于项目的 `logs/motion-verification.json`，启动记录为 `logs/navigation-base.log`。用户已确认 RViz 的机器人、TF 坐标轴和里程计箭头全部显示正常。

解析限制参考：https://github.com/ros/sdformat_urdf/tree/rolling/sdformat_urdf
