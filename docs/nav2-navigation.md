# Nav2 定点导航

导航作为独立 `robot-nav2` 容器运行，复用 `robot-sim-gui` 中正在运行的 Gazebo、雷达、停车保护和 SLAM。该模式在线建图导航，不启动 AMCL 或另一个 map→odom 发布者。镜像、工程与结果保存在 `/mnt/robot_disk`。

## 启动

```bash
docker build -t robot-sim:lyrical-navigation-v1 -f docker/Dockerfile.navigation docker
./docker/start-nav2.sh
```

前提：室内仿真和 SLAM 已启动，`/map`、`map → vehicle/base_link`、`/scan`、`/model/vehicle/odometry` 正常，且没有其他运动脚本或遥控器发布指令。新启动仿真时必须重新建图或正确恢复 SLAM 状态；不能假定之前地图自动载入。

导航用 Smac 2D 规划器、Regulated Pure Pursuit 控制器、Nav2 默认重规划/恢复行为树。规划不允许穿过未知栅格。局部与全局代价地图都接入雷达；车体多边形相对轮轴坐标包含偏置车身及车轮，并留 5 cm padding。膨胀半径 3.3 m、衰减系数 1.0 用于鼓励路径远离障碍。

控制器最高线速度 0.22 m/s、角速度 0.30 rad/s。控制与恢复行为输出 Twist 到原 `/model/vehicle/cmd_vel`，由独立 `velocity_guard` 执行超时、雷达失效与 1.9 m 近障碍停车，然后桥接至 Gazebo。保护不会主动绕障，也可能阻止过于贴近障碍的规划；不要绕过它来让测试通过。

## 单目标实验

```bash
docker exec robot-nav2 bash -lc 'source /opt/ros/lyrical/setup.bash; python3 /work/robot_ws/navigation_base/navigation/check_goal.py --x 10 --y 5 --yaw 0 --output /work/robot_ws/maps/nav2-goal-new.json'
```

坐标为当前 `map` frame，角度单位 rad。该命令先检查规划再发送一个 NavigateToPose 目标，不提供途中航点。`--plan-only` 只检查规划，不驱动车辆；`--expect-failure` 验证导航明确失败且车辆停止，用于不可达测试。输出文件拒绝覆盖。程序记录规划路径、执行轨迹、雷达间距、速度、Nav2 结果码、目标位置和角度误差。真值仅用于记录，不参与控制。

成功判据：Nav2 action SUCCEEDED、地图坐标终点误差 <0.20 m、方向误差 <0.15 rad、实测线/角速度绝对值 <0.02。墙内目标实验要求规划与导航均返回预期错误码（默认 206 / GOAL_OCCUPIED）、导航 action ABORTED、实际位移 <0.05 m 且停稳；不能把任意超时或脚本异常当作通过。

RViz 配置为 `workspace/navigation_base/navigation/navigation.rviz`，新增绿色全局路径和局部代价地图。2D Goal Pose 目标通过 `/goal_pose` 进入导航节点；使用它时不要同时运行验收脚本。

停止导航：`docker stop robot-nav2`。仿真和 SLAM 留在另一个容器中，导航指令失效后由停车保护停止车辆。

参考：[Nav2 Lyrical 默认参数](https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_bringup/params/nav2_params.yaml)、[默认导航行为树](https://github.com/ros-navigation/navigation2/blob/lyrical/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)、[RPP 控制器](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)。

## 2026-10-03 实际验收

初始位置为上一轮建图结束后的 map 约 (0.102,-0.235)，保持 SLAM 在线；每项仅提交终点和朝向，没有向 Nav2 提供中间航点。

| 实验 | map 目标 (x,y,yaw) | 结果 | 终点位置误差 | 最小雷达间距 | 墙钟耗时 |
|---|---|---|---:|---:|---:|
| 穿通道 | (10,5,0) | SUCCEEDED / 0 | 0.145 m | 2.339 m | 163.5 s |
| 绕隔墙 | (10,0,π) | SUCCEEDED / 0 | 0.147 m | 2.338 m | 150.5 s |
| 墙内目标 | (7.5,2.5,0) | ABORTED / 206 GOAL_OCCUPIED | 不适用 | 2.543 m | 6.5 s |
| 拒绝后继续导航 | (9,0,π) | SUCCEEDED / 0 | 0.143 m | 2.488 m | 21.1 s |

三次有效目标均通过位置 <0.20 m、方向 <0.15 rad、停稳阈值；终点方向误差分别约 1.68°、4.07°、4.44°。墙内目标的规划与导航都返回 206，真值位移 0 m，未产生运动。四项结束时线/角速度均为 0。没有通过停用保护或降低阈值使测试通过。

第一条初始规划路径约 14.4 m，第二条约 13.1 m。第二个目标虽然与起点直线距离约 5 m，但被隔墙阻断，Nav2 自行选择从东端绕行。全局地图、局部代价地图、控制器与行为树均保持运行，RViz 中已目视核对路径及代价地图可见。

本轮证明的是静态室内地图上的单目标规划执行、已知隔墙绕行、墙内目标的明确拒绝及失败后继续接受有效目标。所有案例的恢复次数均为 0；不等于已验证动态障碍突然出现、卡住后的 Spin/BackUp 恢复或所有类型不可达目标。

记录位于数据盘 `workspace/maps/nav2-{corridor,detour,unreachable,after-rejection}-20261003.json` 与各自 `.trace.json`。总览为 `nav2-validation-20261003.results.json` 和 `.png`；同期地图为同名前缀 `.yaml`、`.pgm`。导航和构建日志在本地 `logs/nav2-*.log`、`logs/navigation-image-build.log`。

已知日志：Smac 首次配置出现一次非圆形 footprint 膨胀优化警告；实际查询膨胀半径为 3.3 m、footprint 为本配置，所有规划完成。控制循环偶有 10 Hz 未达标警告，需要在性能实验中测量，不把本轮成功归结为实时性能已验收。原有 RViz 首次地图纹理的 sampler 日志仍可出现，界面已确认可见。
