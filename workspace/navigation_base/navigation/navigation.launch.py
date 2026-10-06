from pathlib import Path
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    params=str(Path(__file__).with_name('nav2.yaml'))
    import sys,yaml
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from safety_profile import FOOTPRINT_MODE, PROFILE
    from robot_contract import nav2_parameters, CONTRACT_HASH
    config=nav2_parameters(yaml.safe_load(Path(params).read_text()),PROFILE)
    if FOOTPRINT_MODE:
        follow=config['controller_server']['ros__parameters']['FollowPath']
        follow.update(rotate_to_heading_angular_vel=.10,regulated_linear_scaling_min_speed=.04,
                      min_approach_linear_velocity=.025)
        # Inflation is a graded planning cost; polygon collision remains active.
        for name in ('local_costmap','global_costmap'):
            config[name][name]['ros__parameters']['inflation_layer'].update(inflation_radius=1.2,cost_scaling_factor=3.)
        follow.update(inflation_cost_scaling_factor=3.,cost_scaling_dist=1.0)
    config=nav2_parameters(config,PROFILE)
    runtime=Path(__file__).resolve().parents[2]/'log'/('nav2-contract-'+CONTRACT_HASH[:12]+'.yaml')
    runtime.parent.mkdir(parents=True,exist_ok=True)
    runtime.write_text(yaml.safe_dump(config));params=str(runtime)
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
