"""Log the fake sun sensors to CSV until a given time of day, then exit.

    ros2 run sun_bot sun_logger --ros-args -p out:=panel_east.csv -p label:=east -p end_hour:=17.0

One row per /sun/panel_current message: sim time, time of day, sun angles, panel current, edge
lux, cloudy flag, ground-truth panel fraction and robot pose, sun_brain state, plant moisture.
Plot with `sun_report --panel ...`, `--day-run ...` or `--compare ...`.
"""
import csv
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, String


class SunLogger(Node):

    def __init__(self):
        super().__init__('sun_logger', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.out = self.declare_parameter('out', 'sun_log.csv').value
        self.label = self.declare_parameter('label', '').value
        self.end_hour = self.declare_parameter('end_hour', 17.0).value
        self.hours = self.angles = self.lux = self.pose = None
        self.cloudy = False
        self.frac = 0.0
        self.state = ''
        self.moisture = float('nan')
        self.thirsty = False
        self.done = False
        self.f = open(self.out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['label', 'sim_time', 'hours', 'azimuth', 'elevation', 'panel_current',
                         'lux_front', 'lux_back', 'lux_left', 'lux_right', 'cloudy',
                         'x', 'y', 'yaw_deg', 'panel_fraction', 'state', 'moisture', 'thirsty',
                         'state_detail'])
        self.rows = 0
        self.create_subscription(Float64, '/sun/clock_hours',
                                 lambda m: setattr(self, 'hours', m.data), 10)
        self.create_subscription(Float32MultiArray, '/sun/angles',
                                 lambda m: setattr(self, 'angles', list(m.data)), 10)
        self.create_subscription(Float32MultiArray, '/sun/edge_lux',
                                 lambda m: setattr(self, 'lux', list(m.data)), 10)
        self.create_subscription(Bool, '/sun/cloudy',
                                 lambda m: setattr(self, 'cloudy', m.data), 10)
        self.create_subscription(Odometry, '/sun_bot/ground_truth', self.on_pose, 10)
        self.create_subscription(Float32, '/sun/panel_fraction',
                                 lambda m: setattr(self, 'frac', m.data), 10)
        self.create_subscription(String, '/sun_brain/state',
                                 lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(Float32, '/plant/moisture',
                                 lambda m: setattr(self, 'moisture', m.data), 10)
        self.create_subscription(Bool, '/plant/thirsty',
                                 lambda m: setattr(self, 'thirsty', m.data), 10)
        self.create_subscription(Float32, '/sun/panel_current', self.on_current, 10)
        self.get_logger().info(f'logging to {self.out} until {self.end_hour:g} h')

    def on_pose(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, math.degrees(yaw))

    def on_current(self, m):
        if self.done or None in (self.hours, self.angles, self.lux, self.pose):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self.w.writerow([self.label, f'{t:.2f}', f'{self.hours:.4f}', f'{self.angles[0]:.2f}',
                         f'{self.angles[1]:.2f}', f'{m.data:.4f}',
                         *(f'{v:.0f}' for v in self.lux), int(self.cloudy),
                         *(f'{v:.3f}' for v in self.pose), f'{self.frac:.3f}',
                         self.state.split(' ')[0], f'{self.moisture:.2f}', int(self.thirsty),
                         self.state])
        self.rows += 1
        if self.rows % 50 == 0:
            self.f.flush()
        if self.hours >= self.end_hour:
            self.done = True
            self.get_logger().info(f'{self.rows} rows written to {self.out}')


def main():
    rclpy.init()
    node = SunLogger()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
