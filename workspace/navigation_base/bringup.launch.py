from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition


def generate_launch_description():
    root = Path(__file__).resolve().parent
    gui_env = {'LIBGL_ALWAYS_SOFTWARE': LaunchConfiguration('gui_software'),
               '__GLX_VENDOR_LIBRARY_NAME': LaunchConfiguration('gui_gl_vendor')}
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('gui_software', default_value='1'),
        DeclareLaunchArgument('gui_gl_vendor', default_value='mesa'),
        DeclareLaunchArgument("rviz_config", default_value=str(root / "vehicle.rviz")),
        DeclareLaunchArgument("world", default_value=str(root / "vehicle.world.sdf"), description="SDF world path (must use world name demo and model vehicle)"),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '3', LaunchConfiguration('world')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '3', '--render-engine', 'ogre'], additional_env=gui_env, condition=UnlessCondition(LaunchConfiguration('headless')), output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='vehicle_bridge', output='screen', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/demo/model/vehicle/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/vehicle/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ], remappings=[('/world/demo/model/vehicle/joint_state','/joint_states'),('/model/vehicle/tf','/tf'),('/model/vehicle/cmd_vel','/model/vehicle/cmd_vel_safe')], parameters=[{'use_sim_time':True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{
            'use_sim_time':True, 'frame_prefix':'vehicle/',
            'robot_description':(root / 'vehicle.description.sdf').read_text(),
        }]),
        # DiffDrive integrates at the wheel axle. Keep the physical chassis offset.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_chassis_static_tf',
             arguments=['--x', '-0.7057095', '--y', '0', '--z', '0.5',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'vehicle/base_link', '--child-frame-id', 'vehicle/chassis'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.4',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'vehicle/chassis',
                '--child-frame-id', 'vehicle/lidar',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(package='rviz2', executable='rviz2', arguments=['-d', LaunchConfiguration('rviz_config')], additional_env=gui_env, condition=UnlessCondition(LaunchConfiguration('headless')), parameters=[{'use_sim_time':True}], output='screen'),
    ])
