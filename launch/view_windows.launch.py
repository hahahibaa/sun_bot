"""Show the saved warehouse map with the windows and parking spots (no simulation needed).

    ros2 launch sun_bot view_windows.launch.py
    ros2 launch sun_bot view_windows.launch.py map:=/path/to/other.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sun_bot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg, 'maps', 'warehouse.yaml'),
            description='Map yaml saved by map_saver_cli'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(pkg, 'rviz', 'map_view.rviz'),
            description='RViz config (default: top-down view of the map)'),

        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server']}]),

        Node(package='sun_bot', executable='window_markers', output='screen'),

        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
