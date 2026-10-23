"""Publish the windows and parking spots from config/windows.yaml as a latched MarkerArray.

    ros2 run sun_bot window_markers                      # uses the installed windows.yaml
    ros2 run sun_bot window_markers --ros-args -p windows_file:=/path/to/windows.yaml

Topic: /window_markers (visualization_msgs/MarkerArray, transient local, so RViz gets it
whenever it subscribes). Per window: the opening drawn on the wall, an arrow at the parking
spot pointing at the window, and a text label.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class WindowMarkers(Node):

    def __init__(self):
        super().__init__('window_markers')
        default_file = os.path.join(
            get_package_share_directory('sun_bot'), 'config', 'windows.yaml')
        path = self.declare_parameter('windows_file', default_file).value
        with open(path) as f:
            cfg = yaml.safe_load(f)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'window_markers', qos)
        self.pub.publish(self.build(cfg))
        names = ', '.join(w['name'] for w in cfg['windows'])
        self.get_logger().info(f'published {len(cfg["windows"])} windows ({names}) from {path}')

    def build(self, cfg):
        frame = cfg.get('frame_id', 'map')
        width = cfg.get('window_width', 2.0)
        sill, top = cfg.get('sill_height', 1.0), cfg.get('top_height', 2.2)
        markers = MarkerArray()

        def marker(ns, i, mtype):
            m = Marker()
            m.header.frame_id = frame
            m.ns, m.id, m.type, m.action = ns, i, mtype, Marker.ADD
            return m

        for i, w in enumerate(cfg['windows']):
            cx, cy = w['centre']
            p = w['parking']

            # the window opening: a thin light-blue pane in the wall
            pane = marker('window', i, Marker.CUBE)
            pane.pose.position.x, pane.pose.position.y = float(cx), float(cy)
            pane.pose.position.z = (sill + top) / 2
            (pane.pose.orientation.x, pane.pose.orientation.y,
             pane.pose.orientation.z, pane.pose.orientation.w) = yaw_to_quaternion(
                w['facing_yaw'])
            pane.scale.x, pane.scale.y, pane.scale.z = 0.05, width, top - sill
            pane.color.r, pane.color.g, pane.color.b, pane.color.a = 0.4, 0.8, 1.0, 0.8
            markers.markers.append(pane)

            # parking spot: arrow from the spot towards the window
            arrow = marker('parking', i, Marker.ARROW)
            yaw = p['yaw']
            start = Point(x=float(p['x']), y=float(p['y']), z=0.05)
            end = Point(x=start.x + 0.6 * math.cos(yaw), y=start.y + 0.6 * math.sin(yaw), z=0.05)
            arrow.points = [start, end]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.08, 0.2, 0.2
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.85, 0.0, 1.0
            markers.markers.append(arrow)

            disc = marker('parking_spot', i, Marker.CYLINDER)
            disc.pose.position.x, disc.pose.position.y = start.x, start.y
            disc.pose.position.z = 0.01
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 0.5
            disc.scale.z = 0.02
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = 1.0, 0.85, 0.0, 0.5
            markers.markers.append(disc)

            label = marker('label', i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y = start.x, start.y
            label.pose.position.z = 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = w['name']
            markers.markers.append(label)
        return markers


def main():
    rclpy.init()
    node = WindowMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
