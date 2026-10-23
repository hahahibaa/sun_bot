"""One simulated day: sim + Nav2 + fake sun + plant + sun_brain (+ optional CSV log).

    ros2 launch sun_bot sun_day.launch.py mode:=learn gui:=false rviz:=false   # day 1: learn
    ros2 launch sun_bot sun_day.launch.py                                        # day 2: chase
    ros2 launch sun_bot sun_day.launch.py clouds:=true seed:=7 log:=/tmp/day2.csv
    ros2 launch sun_bot sun_day.launch.py strategy:=parked     # or greedy (baselines)
    ros2 launch sun_bot sun_day.launch.py demo:=true           # scripted demo for the video

demo:=true: the normal-speed day (demo_speed, default 1 day-min per sim-s: 07:00-17:45 takes
~11 min; faster days make the patches outrun the robot), clean RViz layout (rviz/demo.rviz)
with a live panel-current chart, scripted 30-min cloud at ~11:00 while tracking, scripted
drying so the plant honks ~14:00 and is watered at 15:00, log to docs/demo_run.csv.
scripts/record_demo.sh records it as a 5x time-lapse (~2.2 min of video).

The sun clock starts frozen; sun_brain starts it (at start_hour, day_speed) once Nav2 and AMCL
are up, so every run starts its day at the same moment of robot readiness. With the same
seed, start_hour and day_speed, every run gets the same clouds at the same day-times.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    pkg = get_package_share_directory('sun_bot')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    demo = arg('demo').lower() in ('1', 'true', 'yes')
    day_speed = float(arg('demo_speed') if demo else arg('day_speed'))
    log = arg('log') or (os.path.expanduser('~/sun_ws/docs/demo_run.csv') if demo else '')
    gui = arg('gui')
    rviz = arg('rviz').lower() in ('1', 'true', 'yes')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav.launch.py')),
            launch_arguments={'gui': gui,
                              'rviz': 'false' if demo else str(rviz).lower()}.items()),
        Node(package='sun_bot', executable='sun_sensor', output='screen',
             parameters=[{'start_hour': float(arg('start_hour')), 'speed': 0.0,
                          'clouds': (arg('clouds').lower() == 'true') and not demo,
                          'seed': int(arg('seed')),
                          'cloud_duration': 30.0 if demo else 60.0}]),
        Node(package='sun_bot', executable='plant_monitor', output='screen',
             parameters=[{'dry_scale': float(arg('dry_scale')),
                          'beep': arg('beep').lower() == 'true'}]),
        Node(package='sun_bot', executable='sun_brain', output='screen',
             parameters=[{'mode': arg('mode'), 'strategy': arg('strategy'),
                          'start_hour': float(arg('start_hour')), 'day_speed': day_speed,
                          'marker_scale': 0.45 if demo else 0.3}]),
    ]
    if log:
        actions.append(Node(package='sun_bot', executable='sun_logger', output='screen',
                            parameters=[{'out': log, 'label': 'demo' if demo else arg('strategy'),
                                         'end_hour': float(arg('end_hour'))}]))
    if demo:
        actions += [
            Node(package='sun_bot', executable='demo_director', output='screen'),
            Node(package='rviz2', executable='rviz2', output='log',
                 arguments=['-d', os.path.join(pkg, 'rviz', 'demo.rviz')],
                 parameters=[{'use_sim_time': True}]),
        ]
    return actions


def generate_launch_description():
    def declare(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    return LaunchDescription([
        declare('gui', 'true', 'Gazebo GUI'),
        declare('rviz', 'true', 'RViz'),
        declare('mode', 'run', 'learn (day 1) or run (day 2+)'),
        declare('strategy', 'brain', 'brain | parked | greedy'),
        declare('start_hour', '7.0', 'day-time when the day starts'),
        declare('day_speed', '1.0', 'day-minutes per sim-second'),
        declare('clouds', 'false', 'random clouds'),
        declare('seed', '7', 'random seed (clouds, sensor noise)'),
        declare('dry_scale', '1.0', 'plant drying speed-up'),
        declare('beep', 'false', 'plant honks with a real sound (off by default)'),
        declare('log', '', 'CSV file for sun_logger ("" = no log)'),
        declare('end_hour', '17.75', 'sun_logger stops here'),
        declare('demo', 'false', 'scripted demo day for the video'),
        declare('demo_speed', '1.0', 'day-min per sim-s in demo mode'),
        OpaqueFunction(function=setup),
    ])
