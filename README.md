# ros2-gazebo-robot-lab

基于 Docker 的 ROS 2 Lyrical 与 Gazebo 小车仿真实验：速度控制、里程计反馈、TF 和 RViz 可视化，后续逐步接入激光雷达、SLAM 和导航。

## 当前进度

- [x] Docker 仿真环境与持久化目录
- [x] 小车前进、转向、停止
- [x] `/model/vehicle/odometry` 运动反馈
- [x] 修复 `robot_state_publisher` 模型解析错误
- [x] TF 与 RViz 机器人、里程计显示
- [ ] 激光雷达与 `/scan`
- [ ] SLAM 建图
- [ ] Nav2 自动导航

本项目是已验证的基础仿真实验，不是已完成的自主导航系统。当前优先使用 **CPU 软件渲染**；NVIDIA 硬件加速尚未解决。

## 验证环境与前提

已在 Ubuntu 26.04、x86_64、ROS 2 Lyrical、Gazebo Sim 10.5.0 的本地桌面环境验证。需要 Docker、可用的 X11/XWayland 桌面和宿主机 `xauth`。

**当前启动脚本保留已验证的 `--gpus all` 参数，因此还需要 NVIDIA GPU 与 NVIDIA Container Toolkit，尽管实际绘图强制使用 llvmpipe。** 尚未验证无 NVIDIA 主机、Windows/macOS 或纯无头环境。镜像基础标签和 apt 包未锁定版本，未来重新构建的依赖可能变化。

## 首次使用

```bash
git clone https://github.com/huam54925-cpu/ros2-gazebo-robot-lab.git
cd ros2-gazebo-robot-lab

docker build -t robot-sim:lyrical-v1 -f docker/Dockerfile .
./docker/prepare-display.sh
./docker/start-navigation-base.sh
```

若缺少宿主机 `xauth`，Ubuntu 可安装 `sudo apt install xauth`。在当前图形桌面的终端执行显示准备脚本；重新登录桌面后可能需要再次执行。不要在仿真运行过程中刷新授权文件。

脚本自动按仓库位置定位挂载目录，克隆到移动硬盘也可使用。此台机器原项目位置为 `/mnt/robot_disk/ros_sim`。镜像本体通过 Docker 构建，不存进 Git。

启动后会打开 Gazebo 与 RViz，并自动运行物理仿真。节点使用仿真时间。关闭 Gazebo GUI 不会停止独立的服务端；在启动终端按 Ctrl+C，或执行：

```bash
docker stop robot-sim-gui
```

容器自动删除，本地 `home/` 与 `workspace/` 保留。

## 控制与验证

下面的自动验证会让**仿真小车短暂前进和转向，然后停止**：

```bash
docker exec robot-sim-gui bash -lc \
  'source /opt/ros/lyrical/setup.bash; python3 /work/robot_ws/navigation_base/verify_motion.py'
```

输出包括位移、转角、停止速度、关节名、TF 查询结果和 `passed`。软件渲染速度可能影响固定墙钟时长内的仿真位移。

持续读取里程计：

```bash
docker exec -it robot-sim-gui bash -lc \
  'source /opt/ros/lyrical/setup.bash; ros2 topic echo /model/vehicle/odometry'
```

速度话题是 `/model/vehicle/cmd_vel`，消息类型 `geometry_msgs/msg/Twist`；停止命令为全部分量置零。

## 坐标关系与修复

```text
vehicle/odom
└── vehicle/chassis
    ├── vehicle/left_wheel
    ├── vehicle/right_wheel
    └── vehicle/caster
```

DiffDrive 提供唯一里程计与第一段 TF；`robot_state_publisher` 使用关节状态发布车体内部 TF。速度桥接仅 ROS→Gazebo，其余反馈仅 Gazebo→ROS，避免重复发布。

ROS 描述副本去除解析插件不支持的模型级 `<pose>`；Gazebo 物理模型保留自己的位置。球形后轮在 ROS 描述里用固定关节显示，在物理仿真中仍为球关节。平面里程计的 z=0 表示起始车体参考，不表示地面高度。KDL 根链接惯量警告目前仍存在，但模型初始化和 TF 验证已通过。

详见 [配置说明](workspace/navigation_base/README.md) 和 [验证记录](docs/validation.md)。

## 文件结构

- `docker/`：Dockerfile、显示准备、启动脚本。
- `workspace/navigation_base/`：启动文件、模型、场景、RViz 与验证脚本。
- `docs/`：验证结果。
- `third_party/`：上游来源与许可文本。
- `gui/`、`home/`、`logs/`、`data/`：本地运行目录，不上传。

## 上游来源

模型、场景和 RViz 配置源自 [ros_gz_sim_demos](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)，遵循 Apache-2.0；原始副本、修改说明与许可文本随仓库保留。详见 [上游说明](third_party/README.md)。
