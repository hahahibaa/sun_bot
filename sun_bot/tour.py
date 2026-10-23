"""Drive a fixed mapping route around the warehouse, so every wall and pallet face gets scanned.

    ros2 run sun_bot tour

Follows WAYPOINTS (map frame = Gazebo world frame, the robot spawns at 0,0 yaw 0) using the
SLAM-corrected pose map -> base_link from TF, so it needs slam_toolbox (or AMCL) running.
Not /odom: wheel odometry drifted ~2 m over one lap in testing.
The route stays >= 1 m from every pallet; the lidar is only used as a safety stop.
Stops the robot and exits when the last waypoint is reached (or on Ctrl+C).
"""
import math
import signal

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

# (x, y) in m: down to the south side, once around the room ~1.5 m from the walls (sees the
# south faces of all pallets at y = -4.5 and the north faces at y = +4.5), then along the middle
# (y = 0, >= 1.5 m from every pallet) so the open floor gets close-range rays too, then back.
WAYPOINTS = [
    (0.0, -4.5),
    (8.5, -4.5),
    (8.5, 4.5),
    (-8.5, 4.5),
    (-8.5, -4.5),
    (-8.5, 0.0),
    (8.0, 0.0),
    (0.0, 0.0),
]

SPEED = 0.35        # m/s
TURN_SPEED = 0.8    # rad/s, turning in place
REACHED = 0.25      # m
STOP_DIST = 0.35    # m, lidar safety stop (front sector)


class Tour(Node):

    def __init__(self):
        super().__init__('tour', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.step)
        self.i = 0
        self.front = math.inf
        self.finished = False

    def on_scan(self, scan):
        front = [r for k, r in enumerate(scan.ranges)
                 if abs(scan.angle_min + k * scan.angle_increment) < math.radians(25)
                 and math.isfinite(r)]
        self.front = min(front) if front else math.inf

    def step(self):
        if self.finished:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException as e:
            self.get_logger().info(f'waiting for map -> base_link: {e}', throttle_duration_sec=2.0)
            return
        p = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        gx, gy = WAYPOINTS[self.i]
        dist = math.hypot(gx - p.x, gy - p.y)
        cmd = Twist()
        if dist < REACHED:
            self.get_logger().info(
                f'waypoint {self.i + 1}/{len(WAYPOINTS)} ({gx:+.1f}, {gy:+.1f}) reached')
            self.i += 1
            if self.i == len(WAYPOINTS):
                self.finished = True
                self.get_logger().info('route finished, robot stopped')
                self.pub.publish(cmd)
                return
        else:
            err = math.atan2(gy - p.y, gx - p.x) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            if abs(err) > 0.3:
                cmd.angular.z = math.copysign(TURN_SPEED, err)
            elif self.front < STOP_DIST:
                self.get_logger().warn('obstacle ahead, waiting', throttle_duration_sec=2.0)
            else:
                cmd.linear.x = SPEED
                cmd.angular.z = 1.5 * err
        self.pub.publish(cmd)


def main():
    # keep the ROS context alive on Ctrl+C so the final stop command still gets published
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = Tour()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop the robot
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
