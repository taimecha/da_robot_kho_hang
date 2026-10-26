"""Three AMCL instances on the LiDAR-built map; each owns its map->odom TF."""
from pathlib import Path
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from grid_board import Board


def nodes(context):
    folder=Path(LaunchConfiguration('config_dir').perform(context))
    board=Board.load(folder/'grid.yaml')
    source=Path(__file__).resolve().parents[1]/'src/my_bot_bringup/config/navigation.yaml'
    template=yaml.safe_load(source.read_text())['amcl']['ros__parameters']
    result=[]
    for i,name in enumerate(('agv1','agv2','agv3')):
        x,y=board.xy(i+1,0)
        cfg=dict(template,scan_topic='scan',map_topic='/map',use_sim_time=False,
                 set_initial_pose=True,initial_pose=dict(x=x,y=y,z=0.,yaw=board.yaw))
        tf=[('/tf','tf'),('/tf_static','tf_static')]
        result.append(Node(package='nav2_amcl',executable='amcl',name='amcl',
                           namespace=name,parameters=[cfg],remappings=tf,output='screen'))
        result.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                           name='lifecycle_manager_localization',namespace=name,
                           parameters=[{'use_sim_time':False,'autostart':True,
                                        'node_names':['amcl']}],output='screen'))
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir'),OpaqueFunction(function=nodes)])
