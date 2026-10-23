"""Sim + slam_toolbox (online async) + RViz: build a map of the warehouse.

    ros2 launch sun_bot mapping.launch.py
    ros2 launch sun_bot mapping.launch.py gui:=false rviz:=false

The robot spawns at (0, 0, yaw 0), so odom starts at the Gazebo world origin and the map
frame (= first odom pose) lines up with the world frame.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sun_bot')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'slam_params', default_value=os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox parameter file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[LaunchConfiguration('slam_params'), {'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(pkg, 'rviz', 'sim.rviz'), '-f', 'map'],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
