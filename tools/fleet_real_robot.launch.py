"""Phan laptop cua MOT xe that trong namespace rieng (dung trong tools/fleet_real.sh).

    ros2 launch tools/fleet_real_robot.launch.py ns:=agv2 auto_localize:=true

Trong /<ns>, cay TF rieng /<ns>/tf, /<ns>/tf_static (giong mo phong 2 xe - hai
xe deu co frame odom/base_footprint, chung mot cay thi dam nhau):
    robot_state_publisher  URDF -> base_footprint..lidar_link
    wheel_odometry         /<ns>/joint_states + /<ns>/imu/data -> /<ns>/odom + TF
    amcl                   /<ns>/scan_filtered + ban do CHUNG /map -> map->odom
    lifecycle_manager_localization (chi amcl; map_server chung do fleet_real.sh bat)
Nav2 cua xe bat rieng sau (tools/fleet_nav.launch.py) de bat tung xe mot.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

TF = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def nodes(context):
    ns = LaunchConfiguration('ns').perform(context)
    auto_localize = LaunchConfiguration('auto_localize').perform(context).lower() in ('true', '1')
    wheel_radius = float(LaunchConfiguration('wheel_radius').perform(context))
    # Match the RobotModel to traffic_manager and fleet.rviz: one hue per car.
    viz_color = {'agv1': 'blue', 'agv2': 'orange'}.get(ns, 'orange')

    bringup = get_package_share_directory('my_bot_bringup')
    xacro_file = os.path.join(get_package_share_directory('my_bot_description'),
                              'description', 'robot.urdf.xacro')
    with open(os.path.join(bringup, 'config', 'navigation.yaml')) as f:
        amcl = yaml.safe_load(f)['amcl']['ros__parameters']
    amcl.update({
        'use_sim_time': False,
        'map_topic': '/map',          # ban do CHUNG, khong phai /<ns>/map
        'scan_topic': 'scan_filtered',
        'set_initial_pose': not auto_localize,
    })
    if auto_localize:
        amcl.pop('initial_pose', None)
    else:
        x = float(LaunchConfiguration('x').perform(context))
        y = float(LaunchConfiguration('y').perform(context))
        yaw = float(LaunchConfiguration('yaw').perform(context))
        amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    common = dict(namespace=ns, output='screen', remappings=TF)
    return [
        Node(package='my_bot_bringup', executable='scan_filter.py', **common),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': False, 'robot_description': ParameterValue(
                 Command(['xacro ', xacro_file, ' sim:=false body_color:=', viz_color,
                          ' wheel_color:=', viz_color]), value_type=str)}],
             **common),
        Node(package='my_bot_bringup', executable='wheel_odometry.py', name='wheel_odometry',
             parameters=[{'use_sim_time': False, 'wheel_radius': wheel_radius,
                          'wheel_separation': 0.200, 'yaw_source': 'imu_quaternion'}],
             **common),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[amcl], **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['amcl']}],
             **common),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', description='agv1 | agv2 - khop #define XE'),
        DeclareLaunchArgument('x', default_value='0.0', description='vi tri ban dau khi khong tu dinh vi'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='rad'),
        DeclareLaunchArgument('auto_localize', default_value='false',
                              description='true: cho vi tri ban dau tu LiDAR/map'),
        DeclareLaunchArgument('wheel_radius', default_value='0.03281',
                              description='giong bringup_real.launch.py (do tren xe 1)'),
        OpaqueFunction(function=nodes),
    ])
