from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parent
    return LaunchDescription([
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '3', str(root / 'vehicle.world.sdf')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '3', '--render-engine', 'ogre'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='vehicle_bridge', output='screen', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/demo/model/vehicle/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/vehicle/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ], remappings=[('/world/demo/model/vehicle/joint_state','/joint_states'),('/model/vehicle/tf','/tf')], parameters=[{'use_sim_time':True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{
            'use_sim_time':True, 'frame_prefix':'vehicle/',
            'robot_description':(root / 'vehicle.description.sdf').read_text(),
        }]),
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
        Node(package='rviz2', executable='rviz2', arguments=['-d',str(root/'vehicle.rviz')], parameters=[{'use_sim_time':True}], output='screen'),
    ])
