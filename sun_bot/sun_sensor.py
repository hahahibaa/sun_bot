"""Fake sun for the warehouse: day clock, sun position, sun patches and the robot's light sensors.

    ros2 run sun_bot sun_sensor
    ros2 run sun_bot sun_sensor --ros-args -p start_hour:=11.0 -p speed:=5.0 -p clouds:=true

Plays the role of the physical world, so the sensors use Gazebo's ground-truth robot pose
(/sun_bot/ground_truth), not the robot's own localisation. Everything runs on sim time.

Publishes
  /sun/clock          std_msgs/String               time of day "HH:MM"
  /sun/clock_hours    std_msgs/Float64              time of day in hours (e.g. 7.5)
  /sun/angles         std_msgs/Float32MultiArray    [azimuth, elevation] deg (az from north, cw)
  /sun/cloudy         std_msgs/Bool
  /sun/patches        visualization_msgs/MarkerArray  yellow sun patches on the floor + label
  /sun/panel_current  std_msgs/Float32              A, 10 W panel on top of the robot
  /sun/edge_lux       std_msgs/Float32MultiArray    lux at [front, back, left, right] body edges
  /sun/panel_fraction std_msgs/Float32              share of the panel inside a patch (ground
                                                    truth for evaluation, not for the robot)
Service
  /sun/cloud          std_srvs/Trigger              start a cloud now (cloud_duration sim s)
Parameters can be changed at runtime, e.g. `ros2 param set /sun_sensor start_hour 12.0`
restarts the day clock at 12:00, `... speed 0.0` freezes it.
The Gazebo "sun" light is not moved: in this Fortress setup /world/<w>/light_config answers
true but changes neither the scene nor the rendering (tested), so RViz is the sun view.
"""
from datetime import datetime, timedelta
import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, MultiArrayDimension, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from sun_bot.solar import sun_position, today_in, tz_from_hours
from sun_bot.sun_geometry import (all_patches, body_to_world, inside, load_windows, sun_vector)

# robot body (base_link frame): x -0.28 .. +0.12, y -0.15 .. +0.15, top at z = 0.175
EDGES = {'front': (0.12, 0.0), 'back': (-0.28, 0.0),
         'left': (-0.08, 0.15), 'right': (-0.08, -0.15)}
COLOURS = {'east': (1.0, 0.85, 0.0), 'south': (1.0, 0.7, 0.0), 'west': (1.0, 0.95, 0.3)}


class SunSensor(Node):

    def __init__(self):
        super().__init__('sun_sensor', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        p('latitude', 23.18)                   # Jabalpur
        p('longitude', 79.95)
        p('utc_offset', 5.5)                   # IST
        p('date', '')                          # YYYY-MM-DD, '' = today (in utc_offset)
        p('start_hour', 7.0)                   # time of day when the node starts
        p('speed', 1.0)                        # day-minutes per sim-second
        p('windows_file', os.path.join(
            get_package_share_directory('sun_bot'), 'config', 'windows.yaml'))
        p('frame_id', 'map')
        p('rate', 10.0)                        # sensor publish rate, Hz (sim time)
        p('panel_size', 0.25)                  # m, square panel on top of the body
        p('panel_x', -0.08)                    # panel centre in base_link (= body centre)
        p('sensor_height', 0.18)               # m, panel and edge sensors (body top 0.175)
        p('full_sun_current', 0.55)            # A, panel fully in a patch, sun at zenith
        p('indoor_current', 0.01)              # A, diffuse indoor light
        p('current_noise', 0.003)              # A, std dev
        p('indoor_lux', 300.0)
        p('sun_lux', 100000.0)                 # direct sun, normal incidence
        p('lux_noise', 0.03)                   # relative std dev
        p('clouds', False)                     # random clouds
        p('cloud_every_min_h', 1.5)            # day-hours between random clouds
        p('cloud_every_max_h', 4.0)
        p('cloud_min_s', 30.0)                 # sim s
        p('cloud_max_s', 120.0)
        p('cloud_factor', 0.1)                 # sun strength under a cloud
        p('cloud_duration', 60.0)              # sim s, for the /sun/cloud service
        p('seed', -1)                          # random seed, -1 = random

        g = self.get_parameter
        self.cfg, self.room = load_windows(g('windows_file').value)
        self.tz = tz_from_hours(g('utc_offset').value)
        self.date = self._parse_date(g('date').value)
        self.start_hour = g('start_hour').value
        self.speed = g('speed').value
        seed = g('seed').value
        # separate generators: the cloud schedule must not depend on how many sensor-noise
        # samples were drawn (which depends on the robot), so runs with one seed share clouds
        self.rng = random.Random(None if seed < 0 else seed)
        self.cloud_rng = random.Random(None if seed < 0 else seed + 1000)
        self.t0 = None                         # sim time at start_hour
        self.cloud_until = -math.inf
        self.next_cloud_h = None
        self.pose = None
        self.tick = 0
        self.lit_before = set()

        self.pub_clock = self.create_publisher(String, '/sun/clock', 10)
        self.pub_hours = self.create_publisher(Float64, '/sun/clock_hours', 10)
        self.pub_angles = self.create_publisher(Float32MultiArray, '/sun/angles', 10)
        self.pub_cloudy = self.create_publisher(Bool, '/sun/cloudy', 10)
        self.pub_patches = self.create_publisher(MarkerArray, '/sun/patches', 10)
        self.pub_current = self.create_publisher(Float32, '/sun/panel_current', 10)
        self.pub_lux = self.create_publisher(Float32MultiArray, '/sun/edge_lux', 10)
        # ground truth for evaluation/plots only (sun_brain must not use it)
        self.pub_frac = self.create_publisher(Float32, '/sun/panel_fraction', 10)
        self.create_subscription(Odometry, '/sun_bot/ground_truth', self.on_truth, 10)
        self.create_service(Trigger, '/sun/cloud', self.on_cloud)
        self.add_on_set_parameters_callback(self.on_params)
        self.create_timer(1.0 / g('rate').value, self.step)
        self.get_logger().info(
            f'sun over {g("latitude").value:.2f} N {g("longitude").value:.2f} E on {self.date}, '
            f'day starts {self._hhmm(self.start_hour)}, {self.speed:g} day-min per sim-s')

    # ---------------------------------------------------------------- time
    def _parse_date(self, s):
        return datetime.strptime(s, '%Y-%m-%d').date() if s else today_in(self.tz)

    @staticmethod
    def _hhmm(h):
        h %= 24.0
        return f'{int(h):02d}:{int(round((h % 1) * 60)) % 60:02d}'

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def day_hours(self, now):
        return self.start_hour + self.speed * (now - self.t0) / 60.0

    def on_params(self, params):
        now = self.sim_now()
        for prm in params:
            if prm.name == 'start_hour':
                self.start_hour, self.t0 = float(prm.value), now
                self._reseed_clouds()           # a restarted day gets the same clouds again
            elif prm.name == 'speed':
                if self.t0 is not None:
                    self.start_hour, self.t0 = self.day_hours(now), now
                self.speed = float(prm.value)
            elif prm.name == 'date':
                self.date = self._parse_date(prm.value)
        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------- clouds
    def on_cloud(self, request, response):
        dur = self.get_parameter('cloud_duration').value
        self.cloud_until = self.sim_now() + dur
        response.success = True
        response.message = f'cloud for {dur:.0f} sim s'
        self.get_logger().info(f'cloud requested: {response.message}')
        return response

    def cloud_factor(self, now, hours):
        """Sun strength factor. Random clouds are scheduled in day-hours (not sim time) and the
        generator is re-seeded whenever the day restarts, so runs with the same seed, start
        hour and speed get identical clouds whatever the robot does."""
        g = self.get_parameter
        if g('clouds').value and self.speed > 0:
            if self.next_cloud_h is None:
                self.next_cloud_h = hours + self._cloud_gap_h()
            if hours >= self.next_cloud_h and now >= self.cloud_until:
                dur = self.cloud_rng.uniform(g('cloud_min_s').value, g('cloud_max_s').value)
                self.cloud_until = now + dur
                self.next_cloud_h = hours + dur * self.speed / 60.0 + self._cloud_gap_h()
                self.get_logger().info(f'{self._hhmm(hours)}: random cloud for {dur:.0f} sim s')
        return g('cloud_factor').value if now < self.cloud_until else 1.0

    def _cloud_gap_h(self):
        g = self.get_parameter
        return self.cloud_rng.uniform(g('cloud_every_min_h').value, g('cloud_every_max_h').value)

    def _reseed_clouds(self):
        seed = self.get_parameter('seed').value
        self.cloud_rng = random.Random(None if seed < 0 else seed + 1000)
        self.next_cloud_h = None
        self.cloud_until = -math.inf

    # ---------------------------------------------------------------- sensors
    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def step(self):
        now = self.sim_now()
        if now <= 0.0:
            return                              # no /clock yet
        if self.t0 is None:
            self.t0 = now
        g = self.get_parameter
        hours = self.day_hours(now)
        when = datetime(self.date.year, self.date.month, self.date.day,
                        tzinfo=self.tz) + timedelta(hours=hours)
        az, el = sun_position(when, g('latitude').value, g('longitude').value)
        sun = sun_vector(az, el)
        strength = self.cloud_factor(now, hours) * max(0.0, math.sin(math.radians(el)))
        h = g('sensor_height').value
        patches = all_patches(self.cfg, self.room, sun, h) if el > 0 else {}

        if self.pose is not None:
            self.publish_sensors(patches, strength)
        self.pub_cloudy.publish(Bool(data=now < self.cloud_until))   # same rate as the sensors
        if self.tick % max(1, int(g('rate').value / 2)) == 0:   # ~2 Hz
            self.publish_state(when, hours, az, el, sun, now)
        self.tick += 1

    def publish_sensors(self, patches, strength):
        g = self.get_parameter
        polys = [piece for pieces in patches.values() for piece in pieces]
        # panel: fraction of a 7x7 grid of sample points that is inside a patch
        n, size, px = 7, g('panel_size').value, g('panel_x').value
        hits = 0
        for i in range(n):
            for j in range(n):
                bx = px + size * ((i + 0.5) / n - 0.5)
                by = size * ((j + 0.5) / n - 0.5)
                pt = body_to_world(self.pose, bx, by)
                hits += any(inside(pt, poly) for poly in polys)
        frac = hits / (n * n)
        self.pub_frac.publish(Float32(data=frac))
        current = (frac * g('full_sun_current').value * strength + g('indoor_current').value
                   + self.rng.gauss(0.0, g('current_noise').value))
        self.pub_current.publish(Float32(data=max(0.0, current)))

        lux = Float32MultiArray()
        lux.layout.dim.append(MultiArrayDimension(label='front,back,left,right', size=4, stride=4))
        for bx, by in EDGES.values():
            pt = body_to_world(self.pose, bx, by)
            sunny = any(inside(pt, poly) for poly in polys)
            value = g('indoor_lux').value + (g('sun_lux').value * strength if sunny else 0.0)
            lux.data.append(max(0.0, value * (1 + self.rng.gauss(0.0, g('lux_noise').value))))
        self.pub_lux.publish(lux)

    # ---------------------------------------------------------------- state, markers
    def publish_state(self, when, hours, az, el, sun, now):
        cloudy = now < self.cloud_until
        self.pub_clock.publish(String(data=self._hhmm(hours)))
        self.pub_hours.publish(Float64(data=hours))
        angles = Float32MultiArray(data=[float(az), float(el)])
        angles.layout.dim.append(MultiArrayDimension(label='azimuth,elevation', size=2, stride=2))
        self.pub_angles.publish(angles)

        floor = all_patches(self.cfg, self.room, sun, 0.0) if el > 0 else {}
        if set(floor) != self.lit_before:
            self.get_logger().info(
                f'{self._hhmm(hours)}: sun on {", ".join(sorted(floor)) or "no window"} '
                f'(az {az:.0f}, el {el:.0f})')
            self.lit_before = set(floor)
        self.pub_patches.publish(self.markers(floor, cloudy, hours, az, el))

    def markers(self, floor, cloudy, hours, az, el):
        out = MarkerArray()
        frame = self.get_parameter('frame_id').value
        alpha = 0.25 if cloudy else 0.6
        for i, w in enumerate(self.cfg['windows']):
            name = w['name']
            for ns, mtype in (('sun_patch', Marker.TRIANGLE_LIST),):
                m = Marker()
                m.header.frame_id = frame
                m.ns, m.id = ns, i
                pieces = floor.get(name)
                if not pieces:
                    m.action = Marker.DELETE
                    out.markers.append(m)
                    continue
                m.type, m.action = mtype, Marker.ADD
                m.pose.orientation.w = 1.0
                r, gg, b = COLOURS.get(name, (1.0, 0.85, 0.0))
                m.color.r, m.color.g, m.color.b = r, gg, b
                m.scale.x = m.scale.y = m.scale.z = 1.0
                m.color.a = alpha
                for poly in pieces:                  # lit pieces (pallet shadows cut out)
                    for k in range(1, len(poly) - 1):
                        for x, y in (poly[0], poly[k], poly[k + 1]):
                            m.points.append(self._pt(x, y, 0.02))
                out.markers.append(m)
        label = Marker()
        label.header.frame_id = frame
        label.ns, label.id, label.type = 'sun_label', 0, Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y, label.pose.position.z = 0.0, 6.7, 0.5
        label.pose.orientation.w = 1.0
        label.scale.z = 0.5
        label.color.r = label.color.g = label.color.a = 1.0
        label.color.b = 0.4
        label.text = (f'{self._hhmm(hours)}  sun az {az:.0f} el {el:.0f}'
                      + ('  (cloud)' if cloudy else '') + ('  (night)' if el <= 0 else ''))
        out.markers.append(label)
        return out

    @staticmethod
    def _pt(x, y, z):
        return Point(x=float(x), y=float(y), z=float(z))


def main():
    rclpy.init()
    node = SunSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
