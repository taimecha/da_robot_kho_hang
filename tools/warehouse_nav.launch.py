"""One ROS launch adapter for the three isolated Nav2 component containers."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    source = str(Path(__file__).with_name('fleet_nav.launch.py'))
    actions = [DeclareLaunchArgument('config_dir')]
    for i, name in enumerate(('agv1', 'agv2', 'agv3')):
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(source),
            launch_arguments={
                'namespace': name,
                'params_file': PathJoinSubstitution([
                    LaunchConfiguration('config_dir'), f'navigation_{name}.yaml'])
            }.items())
        actions.append(TimerAction(period=float(i*8), actions=[GroupAction([include])]))
    return LaunchDescription(actions)
