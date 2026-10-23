"""Scripted events and a live chart for the demo run (sun_day.launch.py demo:=true).

Events (by day-time from /sun/clock_hours):
  cloud    the first time sun_brain is in TRACK after cloud_after (or at cloud_latest anyway):
           call /sun/cloud, so the video shows LOST -> holding still
  thirst   steer plant_monitor's dry_scale so moisture crosses 30 % at about thirsty_at,
           then call /plant/water at water_at (honk stops once moisture is above 45 %)
Chart: /demo/chart (MarkerArray, map frame, drawn under the room): panel current vs day-time,
the full-sun current of the panel for reference, and a strip coloured by sun_brain state.
"""
import math
import subprocess

from geometry_msgs.msg import Point
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray, Float64, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

STATE_RGB = {'INIT': (0.7, 0.7, 0.7), 'CHOOSE': (0.6, 0.6, 0.8), 'GO': (0.42, 0.68, 0.84),
             'SETTLE': (0.99, 0.68, 0.42), 'TRACK': (0.19, 0.64, 0.33), 'LOST': (0.39, 0.39, 0.39),
             'WAIT': (0.85, 0.85, 0.85), 'NIGHT': (0.03, 0.19, 0.42)}


class DemoDirector(Node):

    def __init__(self):
        super().__init__('demo_director', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        p('cloud_after', 10.75)
        p('cloud_latest', 11.5)
        p('thirsty_at', 14.0)
        p('water_at', 15.0)
        p('day_start', 7.0)
        p('day_end', 17.0)
        p('chart_x0', -10.0)          # map x of day_start (chart spans the room width)
        p('chart_x1', 10.0)
        p('chart_y0', -9.6)           # map y of 0 A
        p('chart_amps', 0.5)          # current at the top of the chart
        p('chart_height', 2.6)        # m
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.hours = self.state = None
        self.current = 0.0
        self.elev = 0.0
        self.moisture = None
        self.samples = []             # (hours, current, full-sun current, state)
        self.cloud_done = self.water_done = False
        self.create_subscription(Float64, '/sun/clock_hours', self.on_hours, 10)
        self.create_subscription(Float32, '/sun/panel_current',
                                 lambda m: setattr(self, 'current', m.data), 10)
        self.create_subscription(Float32MultiArray, '/sun/angles',
                                 lambda m: setattr(self, 'elev', m.data[1]), 10)
        self.create_subscription(String, '/sun_brain/state',
                                 lambda m: setattr(self, 'state', m.data.split(' ')[0]), 10)
        self.create_subscription(Float32, '/plant/moisture',
                                 lambda m: setattr(self, 'moisture', m.data), 10)
        self.cloud = self.create_client(Trigger, '/sun/cloud')
        self.water = self.create_client(Trigger, '/plant/water')
        self.plant_params = self.create_client(SetParameters, '/plant_monitor/set_parameters')
        self.pub = self.create_publisher(MarkerArray, '/demo/chart', 10)
        self.create_timer(0.5, self.tick)
        self.last_dry_set = None
        self.gz_follow_done = False

    def on_hours(self, m):
        if self.hours is None or m.data >= self.hours:
            self.hours = m.data

    def set_dry(self, scale):
        req = SetParameters.Request(parameters=[ParamMsg(name='dry_scale', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(scale)))])
        self.plant_params.call_async(req)

    def gz_follow(self):
        """Point the Gazebo GUI camera at the robot (behind-left, above) for the video."""
        for svc, typ, req in (('/gui/follow/offset', 'ignition.msgs.Vector3d',
                               'x: -1.6, y: 1.6, z: 1.3'),
                              ('/gui/follow', 'ignition.msgs.StringMsg', 'data: "sun_bot"')):
            subprocess.Popen(['ign', 'service', '-s', svc, '--reqtype', typ, '--reptype',
                              'ignition.msgs.Boolean', '--timeout', '3000', '--req', req],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def tick(self):
        h = self.hours
        if h is None or self.state in (None, 'INIT'):
            return
        if not self.gz_follow_done:
            self.gz_follow_done = True
            self.gz_follow()
        full = 0.55 * max(0.0, math.sin(math.radians(self.elev))) + 0.01
        if h <= self.g('day_end') + 0.75:
            self.samples.append((h, self.current, full, self.state))

        # scripted cloud while tracking, so LOST is visible
        if not self.cloud_done and h >= self.g('cloud_after') and \
                (self.state == 'TRACK' or h >= self.g('cloud_latest')):
            self.cloud_done = True
            self.cloud.call_async(Trigger.Request())
            self.get_logger().info(f'{h:.2f} h: scripted cloud (robot is {self.state})')

        # thirst: steer dry_scale so moisture reaches ~27 % at thirsty_at. The plant dries
        # 1.5 %/h, or 4.5 %/h while the robot is in the sun, times dry_scale.
        if self.moisture is not None and not self.water_done:
            left = self.g('thirsty_at') - h
            if self.moisture < 30.0:
                if self.last_dry_set != -1.0:
                    self.last_dry_set = -1.0
                    self.set_dry(0.2)            # thirsty: keep it below 30 % until watering
            elif left > 0.05 and (self.last_dry_set is None or h - self.last_dry_set > 0.1):
                self.last_dry_set = h
                rate = 4.5 if self.current > 0.05 else 1.5
                self.set_dry(max(0.0, (self.moisture - 27.0) / left / rate))
            elif left <= 0.05 and self.last_dry_set != -2.0:
                self.last_dry_set = -2.0
                self.set_dry(20.0)               # late: dry quickly until it is thirsty
        if not self.water_done and h >= self.g('water_at'):
            self.water_done = True
            self.water.call_async(Trigger.Request())
            self.set_dry(1.0)
            self.get_logger().info(f'{h:.2f} h: scripted watering')
        self.publish_chart()

    # ---------------------------------------------------------------- chart
    def xy(self, hours, amps):
        g = self.g
        k = (hours - g('day_start')) / (g('day_end') - g('day_start'))
        x = g('chart_x0') + k * (g('chart_x1') - g('chart_x0'))
        y = g('chart_y0') + min(amps, g('chart_amps')) / g('chart_amps') * g('chart_height')
        return Point(x=float(x), y=float(y), z=0.05)

    def marker(self, ns, mid, mtype, rgb, a=1.0, scale=0.05):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in (*rgb, a))
        return m

    def text(self, mid, x, y, s, size=0.35, rgb=(0.9, 0.9, 0.9)):
        m = self.marker('chart_text', mid, Marker.TEXT_VIEW_FACING, rgb, scale=size)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.1
        m.text = s
        return m

    def publish_chart(self):
        g = self.g
        out = MarkerArray()
        d0, d1 = g('day_start'), g('day_end')
        axes = self.marker('chart_axes', 0, Marker.LINE_LIST, (0.75, 0.75, 0.75), scale=0.03)
        for a, b in ((self.xy(d0, 0), self.xy(d1, 0)), (self.xy(d0, 0), self.xy(d0, 0.5))):
            axes.points += [a, b]
        for hh in range(int(d0), int(d1) + 1):
            p0, p1 = self.xy(hh, 0), self.xy(hh, 0)
            p1.y -= 0.12
            axes.points += [p0, p1]
            out.markers.append(self.text(100 + hh, p0.x, p0.y - 0.35, f'{hh:02d}', 0.3))
        out.markers.append(axes)
        for k, amps in enumerate((0.1, 0.2, 0.3, 0.4, 0.5)):
            p = self.xy(d0, amps)
            out.markers.append(self.text(200 + k, p.x - 0.45, p.y, f'{amps:.1f}', 0.25))
        for k, (st, rgb) in enumerate(STATE_RGB.items()):        # legend for the state strip
            if st == 'INIT':
                continue
            p = self.xy(d0, 0)
            out.markers.append(self.text(300 + k, p.x + 0.6 + 2.6 * (k - 1), p.y - 1.25, st,
                                         0.3, rgb))
        top = self.xy(d0, 0.5)
        out.markers.append(self.text(1, top.x + 3.2, top.y + 0.35,
                                     'panel current (A)   - - full sun', 0.33))
        if len(self.samples) >= 2:
            full = self.marker('chart_full', 0, Marker.LINE_LIST, (0.8, 0.8, 0.8), 0.7, 0.025)
            cur = self.marker('chart_current', 0, Marker.LINE_STRIP, (0.95, 0.2, 0.2), 1.0, 0.05)
            band = self.marker('chart_state', 0, Marker.LINE_LIST, (1, 1, 1), 1.0, 0.25)
            for i, (h, c, f, st) in enumerate(self.samples):
                if i and (i // 3) % 2 == 0:          # dashed reference line
                    ph, _, pf, _ = self.samples[i - 1]
                    full.points += [self.xy(ph, pf), self.xy(h, f)]
                cur.points.append(self.xy(h, c))
                if i:
                    a = self.xy(self.samples[i - 1][0], 0)
                    b = self.xy(h, 0)
                    a.y = b.y = g('chart_y0') - 0.75
                    rgb = STATE_RGB.get(st, (1, 1, 1))
                    band.points += [a, b]
                    band.colors += [ColorRGBA(r=float(rgb[0]), g=float(rgb[1]),
                                              b=float(rgb[2]), a=1.0)] * 2
            out.markers += [full, cur, band]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DemoDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
