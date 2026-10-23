"""Warehouse world + sun_bot + ROS 2 bridge + robot_state_publisher, in one command.

    ros2 launch sun_bot sim.launch.py            # with Gazebo GUI
    ros2 launch sun_bot sim.launch.py gui:=false # server only

Gazebo server and GUI are started as two processes so they can use different OpenGL
drivers: on WSL the D3D12 GPU driver makes ogre's gpu_lidar return range_min for every
ray, so by default the server (which renders the sensors) uses Mesa's software renderer
(llvmpipe). The GUI keeps the GPU. Both use the ogre engine.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sun_bot')
    world = os.path.join(pkg, 'worlds', 'warehouse.sdf')
    model = os.path.join(pkg, 'models', 'sun_bot', 'model.sdf')

    # robot_state_publisher reads the SDF directly (sdformat_urdf parser plugin)
    with open(model) as f:
        robot_description = f.read()

    # ogre2 crashes on WSL: use ogre everywhere (the world's Sensors plugin also says ogre)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI'),
        DeclareLaunchArgument('server_software_gl', default_value='1',
                              description='1 = Gazebo server renders sensors with llvmpipe '
                                          '(needed for gpu_lidar on WSL), 0 = use the GPU'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='ROS topic that drives the robot (nav.launch.py uses '
                                          '/cmd_vel_safe, the collision monitor output)'),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-s', '-r', world, '--render-engine', 'ogre'],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': LaunchConfiguration('server_software_gl')},
            output='screen', on_exit=Shutdown()),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '--render-engine', 'ogre'],
            output='screen', on_exit=Shutdown(),
            condition=IfCondition(LaunchConfiguration('gui'))),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'warehouse', '-name', 'sun_bot', '-file', model,
                        '-x', '0', '-y', '0', '-z', '0.01']),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                 '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                 '/sun_bot/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                 '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                 '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                 '/world/warehouse/model/sun_bot/joint_state'
                 '@sensor_msgs/msg/JointState[ignition.msgs.Model',
             ],
             remappings=[('/world/warehouse/model/sun_bot/joint_state', '/joint_states'),
                         ('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
             parameters=[{'use_sim_time': True}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}]),
    ])
