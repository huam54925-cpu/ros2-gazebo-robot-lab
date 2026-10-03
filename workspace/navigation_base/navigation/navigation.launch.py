from pathlib import Path
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    params=str(Path(__file__).with_name('nav2.yaml'))
    names=['controller_server','planner_server','behavior_server','bt_navigator']
    packages=['nav2_controller','nav2_planner','nav2_behaviors','nav2_bt_navigator']
    nodes=[Node(package=p,executable=n,name=n,output='screen',parameters=[params],
                remappings=[('cmd_vel','/model/vehicle/cmd_vel')]) for p,n in zip(packages,names)]
    # A failed navigation process stops the other command sources. The separate
    # simulation velocity guard then stops the vehicle on command timeout.
    handlers=[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[
        EmitEvent(event=Shutdown(reason='Navigation process exited'))])) for n in nodes]
    manager=Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',output='screen',parameters=[{
            'use_sim_time':True,'autostart':True,'node_names':names,'bond_timeout':10.0}])
    return LaunchDescription(handlers+nodes+[manager])
