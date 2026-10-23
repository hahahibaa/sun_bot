from glob import glob

from setuptools import find_packages, setup

package_name = 'sun_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/sun_bot', glob('models/sun_bot/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hahahibaa',
    maintainer_email='223480619+hahahibaa@users.noreply.github.com',
    description='Sun-tracking plant robot in a simulated warehouse',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'window_markers = sun_bot.window_markers:main',
            'wander = sun_bot.wander:main',
            'tour = sun_bot.tour:main',
            'go_to_window = sun_bot.go_to_window:main',
            'sun_sensor = sun_bot.sun_sensor:main',
            'sun_report = sun_bot.sun_report:main',
            'sun_logger = sun_bot.sun_logger:main',
            'sun_brain = sun_bot.sun_brain:main',
            'plant_monitor = sun_bot.plant_monitor:main',
            'demo_director = sun_bot.demo_director:main',
            'sun_replay = sun_bot.sun_replay:main',
            'record_window = sun_bot.record_window:main',
        ],
    },
)
