# 2026-10-01 验证记录

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

实际使用 Mesa llvmpipe 软件渲染。尚未验证 NVIDIA 加速、SLAM 或 Nav2。

发布整理时另行检查了 Shell/Python 语法、XML/YAML 可解析性、显示准备脚本和 Git 上传文件范围；未重新构建镜像，也未重新启动已暂停的仿真。
