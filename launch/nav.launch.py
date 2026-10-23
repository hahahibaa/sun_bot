"""Sim + Nav2 (AMCL on the saved map) + collision monitor + window markers + RViz.

    ros2 launch sun_bot nav.launch.py                       # everything, with GUIs
    ros2 launch sun_bot nav.launch.py gui:=false            # no Gazebo GUI (about 2x faster)
    ros2 launch sun_bot nav.launch.py gui:=false rviz:=false

Velocity chain: Nav2 -> /cmd_vel -> collision_monitor -> /cmd_vel_safe -> Gazebo.
nav2_bringup (Humble) does not start the collision monitor, so it is started here with its
own lifecycle manager, and the bridge is told to drive the robot from /cmd_vel_safe.
AMCL starts at (0, 0, 0) by itself (set_initial_pose in nav2_params.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sun_bot')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg, 'maps', 'warehouse.yaml'),
            description='Map yaml'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            description='Nav2 + collision monitor parameters'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='True = all Nav2 nodes in one container. Off by default: on Humble the '
                        'container ignores Ctrl+C and gets SIGKILLed, which can leave DDS stuck'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'),
                              'cmd_vel_topic': '/cmd_vel_safe'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'params_file': params,
                              'use_sim_time': 'true',
                              'autostart': 'true',
                              'slam': 'False',
                              'use_composition': LaunchConfiguration('use_composition')}.items()),

        # started 5 s later: configuring it in the middle of the start-up rush occasionally hung
        # (stuck after "Creating Scan", never activated -> nothing reaches /cmd_vel_safe)
        TimerAction(period=5.0, actions=[
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', output='screen',
                 parameters=[params, {'use_sim_time': True}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_collision_monitor', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['collision_monitor']}]),
        ]),

        Node(package='sun_bot', executable='window_markers', output='screen'),

        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(pkg, 'rviz', 'nav.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
