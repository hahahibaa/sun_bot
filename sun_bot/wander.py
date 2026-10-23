"""Drive sun_bot around using only its lidar: forward when clear, turn away when not.

    ros2 run sun_bot wander                               # until Ctrl+C
    ros2 run sun_bot wander --ros-args -p duration:=300.0  # stop after 300 s of sim time
"""
import math
import signal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class Wander(Node):
    def __init__(self):
        super().__init__('wander', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.turn_dir = 1.0
        self.duration = self.declare_parameter('duration', 0.0).value  # sim s, 0 = forever
        self.start = None
        self.done = False

    def sector_min(self, scan, lo_deg, hi_deg):
        vals = []
        for i, r in enumerate(scan.ranges):
            a = math.degrees(scan.angle_min + i * scan.angle_increment)
            if lo_deg <= a <= hi_deg:
                vals.append(r if math.isfinite(r) else scan.range_max)
        return min(vals) if vals else scan.range_max

    def on_scan(self, scan):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = now
        if self.duration > 0 and now - self.start > self.duration:
            if not self.done:
                self.pub.publish(Twist())
                self.get_logger().info(f'{self.duration:.0f} s done, robot stopped')
                self.done = True
            return
        front = self.sector_min(scan, -35, 35)
        left = self.sector_min(scan, 35, 100)
        right = self.sector_min(scan, -100, -35)
        cmd = Twist()
        if front > 0.9:
            cmd.linear.x = 0.35
            # drift toward the more open side
            cmd.angular.z = 0.4 * (left - right) / max(left + right, 1e-3)
        else:
            if front > 0.6:  # only pick a new direction when the obstacle first appears
                self.turn_dir = 1.0 if left >= right else -1.0
            cmd.angular.z = 0.8 * self.turn_dir
        self.pub.publish(cmd)
        self.get_logger().info(
            f'front {front:4.2f} m  left {left:4.2f} m  right {right:4.2f} m  -> '
            f'v={cmd.linear.x:.2f} w={cmd.angular.z:+.2f}', throttle_duration_sec=1.0)


def main():
    # keep the ROS context alive on Ctrl+C so the final stop command still gets published
    # (the diff drive keeps executing the last /cmd_vel it received)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)  # also if started with SIGINT ignored
    node = Wander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop the robot
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
