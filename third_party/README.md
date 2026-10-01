# Upstream attribution

The vehicle model, world and RViz configuration under `workspace/navigation_base/`
originate from the ROS `ros_gz_sim_demos` package, part of
[ros_gz](https://github.com/gazebosim/ros_gz), licensed under Apache-2.0.
The license text is included in `Apache-2.0.txt`.

Source paths:
- `ros_gz_sim_demos/models/vehicle/model.sdf`
- `ros_gz_sim_demos/worlds/vehicle.sdf`
- `ros_gz_sim_demos/rviz/vehicle.rviz`

The `*.original.sdf` files are unmodified copies from the locally installed ROS 2
Lyrical package. Modified derivatives are `vehicle.description.sdf`,
`vehicle.world.sdf` and `vehicle.rviz`. Their changes are described in the project
README and file comments. `bringup.launch.py` was written for this project based
on the same launch architecture (Gazebo, bridge, state publisher, RViz).

No blanket license is assigned here to the project's new scripts and documentation;
the upstream assets retain their Apache-2.0 license.
