"""sun_brain: keep the plant in the sun.

    ros2 run sun_bot sun_brain --ros-args -p mode:=learn     # day 1: learn where the sun lands
    ros2 run sun_bot sun_brain                               # day 2+: chase the sun
    ros2 run sun_bot sun_brain --ros-args -p strategy:=parked   # baseline: park at south spot
    ros2 run sun_bot sun_brain --ros-args -p strategy:=greedy   # baseline: edge sensors only

States
  LEARN   survey: at each learn hour visit every sunlit window, SEARCH for its patch (inward and
          sideways sweep legs, then climb the edge-lux gradient), record best panel current +
          pose in learned_sun.yaml. The day clock is FROZEN (sun_sensor speed 0) during each
          survey, so the sun does not move while the robot searches (logged).
  CHOOSE  pick the learned spot with the highest expected current at the estimated arrival
          time (linear interpolation between learned hours; unreachable spots skipped).
  GO      Nav2 to it.               SETTLE  creep (<= 0.1 m/s) up the edge-lux gradient until
          the panel gives 80 % of the full-sun current for the current elevation.
  TRACK   every few day-minutes nudge towards the brighter edge; re-settle if the panel drops
          while sun is still seen; back to CHOOSE when every sensor is at indoor level.
  LOST    cloud: everything dropped at once by a similar ratio -> hold up to cloud_hold s.
  NIGHT   sun down: park at the first morning spot from the learned table.
  WAIT    nothing reachable is lit: stay, re-choose in a few day-minutes.
  EXPLORE (greedy baseline) wander until an edge sensor sees sun.   PARKED (parked baseline)

Uses only what the robot has: AMCL pose (TF map -> base_link), /sun/panel_current,
/sun/edge_lux, /scan, the day clock (/sun/clock_hours) and sun angles (/sun/angles, which a
real robot computes from clock + location). Never ground truth, never /sun/cloudy.
All motion goes through Nav2 or /cmd_vel, i.e. through the collision monitor (/cmd_vel_safe).
"""
from collections import deque
from datetime import datetime
import math
import os
import signal

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import numpy as np
from PIL import Image
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
import yaml

from sun_bot.sun_geometry import load_windows

INDOOR_LUX = 300.0
LIDAR_X = -0.03


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def hhmm(h):
    h %= 24.0
    return f'{int(h):02d}:{int(round((h % 1) * 60)) % 60:02d}'


class SunBrain(Node):

    def __init__(self):
        super().__init__('sun_brain', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        share = get_package_share_directory('sun_bot')
        p = self.declare_parameter
        p('mode', 'run')                       # run | learn
        p('strategy', 'brain')                 # brain | parked | greedy
        p('learned_file', os.path.expanduser('~/sun_ws/src/sun_bot/config/learned_sun.yaml'))
        p('windows_file', os.path.join(share, 'config', 'windows.yaml'))
        p('map_file', os.path.join(share, 'maps', 'warehouse.yaml'))
        p('learn_hours', [7.0, 9.0, 11.0, 13.0, 15.0, 17.0])
        p('start_hour', 7.0)                   # day clock (re)started here when ready
        p('day_speed', 1.0)                    # day-min per sim-s after start (run mode)
        p('sun_sensor', '/sun_sensor')
        p('full_sun_current', 0.55)            # A, panel spec at normal incidence
        p('indoor_current', 0.01)
        p('lit_lux', 1500.0)                   # edge sensor "sees sun" above this
        p('settle_fraction', 0.8)
        p('creep_speed', 0.08)                 # m/s (<= 0.1)
        p('turn_speed', 0.35)                  # rad/s while creeping
        p('track_period', 3.0)                 # sim s between TRACK checks (3 day-min at 1x)
        p('cloud_hold', 120.0)                 # sim s
        p('search_inward', 5.5)                # m from the inner wall face
        p('search_max_s', 90.0)                # sim s per window, counted from reaching it
        p('climb_max_s', 45.0)
        p('settle_max_s', 60.0)
        p('park_window', 'south')              # parked strategy
        p('marker_scale', 0.3)                 # m, height of the status text above the robot
        # creeping keeps every obstacle this far from base_link, so the robot can always still turn
        # in place: the collision monitor's stop zone is the footprint rectangle grown by 0.30 m
        # (x -0.58..0.42, y +/-0.50), whose corner is 0.77 m from base_link, + 0.05 m margin.
        # Closer, a turn swings that corner onto the wall and the robot freezes mid-turn.
        p('turn_clearance', 0.82)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g

        self.cfg, self.room = load_windows(g('windows_file'))
        self.windows = {w['name']: w for w in self.cfg['windows']}
        self._load_map(g('map_file'))
        self.learned = None
        if g('mode') == 'run' and g('strategy') == 'brain':
            with open(g('learned_file')) as f:
                self.learned = yaml.safe_load(f)
            self.get_logger().info(f'loaded {len(self.learned["samples"])} learned hours '
                                   f'from {g("learned_file")}')

        # inputs
        self.hours = self.angles = self.current = self.lux = None
        self.moisture = float('nan')
        self.scan = None
        self.hist = deque(maxlen=60)           # (sim t, current, lux) for cloud detection
        self.clock_samples = deque(maxlen=50)  # (sim t, hours) for the day-clock rate
        self.create_subscription(Float64, '/sun/clock_hours', self.on_hours, 10)
        self.create_subscription(Float32MultiArray, '/sun/angles',
                                 lambda m: setattr(self, 'angles', list(m.data)), 10)
        self.create_subscription(Float32, '/sun/panel_current', self.on_current, 10)
        self.create_subscription(Float32MultiArray, '/sun/edge_lux',
                                 lambda m: setattr(self, 'lux', list(m.data)), 10)
        self.create_subscription(Float32, '/plant/moisture',
                                 lambda m: setattr(self, 'moisture', m.data), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        # outputs
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/sun_brain/state', 10)
        self.pub_marker = self.create_publisher(Marker, '/sun_brain/marker', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal = None                   # goal handle
        self.nav_status = 'idle'               # idle | pending | active | succeeded | failed
        self.sun_params = self.create_client(SetParameters, f'{g("sun_sensor")}/set_parameters')
        self.bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.cm_state = self.create_client(GetState, '/collision_monitor/get_state')

        self.state, self.detail = 'INIT', ''
        self.behaviour = self.init_gen()
        self.blacklist = {}                    # window -> day-hour until which it is skipped
        self.target = None
        self.strip_until = -1.0                # day-hour: ignore wall-strip sun here until then
        self.last_probe_h = -1.0               # day-hour of the last look-around
        self.tick_n = 0
        self.create_timer(0.1, self.tick)

    # ================================================================ inputs / helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_hours(self, m):
        self.hours = m.data
        self.clock_samples.append((self.now(), m.data))

    def on_current(self, m):
        self.current = m.data
        if self.lux is not None:
            self.hist.append((self.now(), m.data, list(self.lux)))

    def day_rate(self):
        """Day-hours per sim-second, measured from /sun/clock_hours."""
        if len(self.clock_samples) < 2:
            return 1.0 / 60.0
        (t0, h0), (t1, h1) = self.clock_samples[0], self.clock_samples[-1]
        return (h1 - h0) / (t1 - t0) if t1 - t0 > 0.5 else 1.0 / 60.0

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform
        except TransformException:
            return None
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def elevation(self):
        return self.angles[1] if self.angles else -90.0

    def full_sun(self):
        return self.g('full_sun_current') * max(0.0, math.sin(math.radians(self.elevation())))

    def lit(self):
        return [v > self.g('lit_lux') for v in self.lux]

    def cmd(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub_cmd.publish(t)

    def set_state(self, state, gen, detail=''):
        if state != self.state:
            self.get_logger().info(
                f'[{hhmm(self.hours) if self.hours is not None else "--:--"}] '
                f'{self.state} -> {state}{": " + detail if detail else ""}')
        self.state, self.detail = state, detail
        self.behaviour = gen

    # ================================================================ map
    def _load_map(self, path):
        with open(path) as f:
            meta = yaml.safe_load(f)
        img = np.array(Image.open(os.path.join(os.path.dirname(path), meta['image'])))
        free = img > 250
        self.map_res = meta['resolution']
        self.map_origin = meta['origin'][:2]
        self.clearance = np.flipud(ndimage.distance_transform_edt(free) * self.map_res)

    def clear_at(self, x, y):
        c = int((x - self.map_origin[0]) / self.map_res)
        r = int((y - self.map_origin[1]) / self.map_res)
        if 0 <= r < self.clearance.shape[0] and 0 <= c < self.clearance.shape[1]:
            return float(self.clearance[r, c])
        return 0.0

    # ================================================================ nav2
    def nav_go(self, x, y, yaw):
        self.nav_cancel()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav_status = 'pending'
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(self._nav_accepted)

    def _nav_accepted(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.nav_status = 'failed'
            return
        self.nav_goal = handle
        self.nav_status = 'active'
        handle.get_result_async().add_done_callback(self._nav_done)

    def _nav_done(self, fut):
        if self.nav_goal is None:
            return
        status = fut.result().status
        self.nav_goal = None
        self.nav_status = 'succeeded' if status == GoalStatus.STATUS_SUCCEEDED else 'failed'

    def nav_cancel(self):
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_goal = None
        self.nav_status = 'idle'

    def nav_busy(self):
        return self.nav_status in ('pending', 'active')

    # ================================================================ sun clock control
    def set_sun(self, **values):
        req = SetParameters.Request()
        for k, v in values.items():
            req.parameters.append(ParamMsg(name=k, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))))
        return self.sun_params.call_async(req)

    # ================================================================ main loop
    def tick(self):
        self.tick_n += 1
        if self.tick_n % 5 == 0:
            self.publish_status()
        gen = self.behaviour
        try:
            next(gen)
        except StopIteration:
            if self.behaviour is gen:          # finished without handing over to a new state
                self.behaviour = self.idle_gen()

    def publish_status(self):
        self.pub_state.publish(String(data=f'{self.state} {self.detail}'.strip()))
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns, m.id, m.type, m.action = 'sun_brain', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.z = 0.9
        m.pose.orientation.w = 1.0
        m.scale.z = self.g('marker_scale')
        m.color.r, m.color.g, m.color.b, m.color.a = 0.05, 0.08, 0.35, 1.0   # dark: map is white
        cur = f'{self.current:.2f} A' if self.current is not None else '- A'
        moist = f'plant {self.moisture:.0f}%' if not math.isnan(self.moisture) else ''
        m.text = (f'{self.state}  {hhmm(self.hours) if self.hours is not None else "--:--"}\n'
                  f'{cur}  {moist}')
        self.pub_marker.publish(m)
        if self.target:
            t = Marker()
            t.header.frame_id = 'map'
            t.ns, t.id, t.type, t.action = 'sun_brain_target', 0, Marker.ARROW, Marker.ADD
            x, y, yaw = self.target['pose']
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.05
            t.pose.orientation.z, t.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            t.scale.x, t.scale.y, t.scale.z = 0.5, 0.1, 0.1
            t.color.g, t.color.b, t.color.a = 1.0, 0.3, 1.0
            self.pub_marker.publish(t)

    def ready(self):
        return (None not in (self.hours, self.angles, self.current, self.lux)
                and self.pose() is not None)

    def wait(self, sim_s):
        t_end = self.now() + sim_s
        while self.now() < t_end:
            yield

    # ================================================================ start-up
    def init_gen(self):
        self.detail = 'waiting for Nav2, AMCL and the sun'
        while not (self.nav.server_is_ready() and self.sun_params.service_is_ready()
                   and self.bt_state.service_is_ready()
                   and self.pose() is not None and self.hours is not None):
            yield
        # the action server exists before Nav2 is activated (goals sent earlier are rejected),
        # and nothing moves the robot unless the collision monitor is active (it feeds
        # /cmd_vel_safe), so wait for both to be ACTIVE before starting the day
        t0 = self.now()
        for name, client in (('bt_navigator', self.bt_state),
                             ('collision_monitor', self.cm_state)):
            while True:
                if client.service_is_ready():
                    fut = client.call_async(GetState.Request())
                    while not fut.done():
                        yield
                    if fut.result().current_state.id == 3:      # lifecycle ACTIVE
                        break
                if self.now() - t0 > 30.0:
                    self.get_logger().error(
                        f'{name} is not active after 30 s: the robot cannot move. Restart the '
                        f'launch (Humble start-up occasionally hangs).',
                        throttle_duration_sec=30.0)
                self.detail = f'waiting for {name} to be active'
                yield from self.wait(1.0)
        mode, strategy = self.g('mode'), self.g('strategy')
        if mode == 'learn':
            self.set_state('LEARN', self.learn_gen())
            return
        # start the day for this run (same start for every strategy)
        fut = self.set_sun(speed=self.g('day_speed'), start_hour=self.g('start_hour'))
        while not fut.done():
            yield
        yield from self.wait(1.0)
        while not self.ready():
            yield
        self.get_logger().info(f'day started at {hhmm(self.g("start_hour"))}, '
                               f'{self.g("day_speed"):g} day-min per sim-s, strategy {strategy}')
        if strategy == 'parked':
            self.set_state('GO', self.parked_gen(), self.g('park_window'))
        elif strategy == 'greedy':
            self.set_state('EXPLORE', self.explore_gen())
        else:
            self.set_state('CHOOSE', self.choose_gen())

    def idle_gen(self):
        while True:
            yield

    # ================================================================ creep controller
    def turn_clearance(self, v=0.0, w=0.0, dt=0.5):
        """Distance from base_link to the nearest lidar point, now or after (v, w) for dt s."""
        s = self.scan
        if s is None:
            return math.inf
        th = w * dt
        dx, dy = v * dt * math.cos(th / 2), v * dt * math.sin(th / 2)
        best = math.inf
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r > 2.0:
                continue
            a = s.angle_min + i * s.angle_increment
            best = min(best, math.hypot(LIDAR_X + r * math.cos(a) - dx, r * math.sin(a) - dy))
        return best

    def safe_cmd(self, v, w):
        """Send (v, w) unless it would bring an obstacle closer than turn_clearance to base_link
        (moves that increase the distance are always allowed). Returns False if refused."""
        now_c, next_c = self.turn_clearance(), self.turn_clearance(v, w)
        if next_c < self.g('turn_clearance') and next_c < now_c - 1e-3:
            self.cmd()
            return False
        self.cmd(v, w)
        return True

    def creep_step(self):
        """One step up the edge-lux gradient: 'inside', 'none', 'moving' or 'blocked' (the
        brighter side is too close to a wall/obstacle to creep further)."""
        lit = self.lit()
        if all(lit):
            self.cmd()
            return 'inside'
        if not any(lit):
            self.cmd()
            return 'none'
        top = max(self.lux)
        f, b, left, right = (v / top for v in self.lux)
        gx, gy = f - b, left - right
        v = self.g('creep_speed')
        if math.hypot(gx, gy) < 0.25:
            # balanced (e.g. front + back lit only): rotate to probe for a gradient
            ok = self.safe_cmd(0.0, self.g('turn_speed'))
            return 'moving' if ok else 'blocked'
        a = math.atan2(gy, gx)
        if abs(a) <= 0.5:
            ok = self.safe_cmd(v, 0.8 * a)
        elif abs(a) >= math.pi - 0.5:
            ok = self.safe_cmd(-v, 0.8 * wrap(a - math.pi))
        else:
            ok = self.safe_cmd(0.0, math.copysign(self.g('turn_speed'), a))
        return 'moving' if ok else 'blocked'

    def cloud_event(self):
        """True if the readings just dropped together, like a cloud (not a moving patch).

        A patch edge moves at a few cm per sim-s, so it uncovers the sensors one by one over
        tens of seconds. A cloud cuts every lit reading at once by about the same ratio."""
        if len(self.hist) < 12:
            return False
        t_now, cur, lux = self.hist[-1]
        old = next((h for h in self.hist if t_now - h[0] <= 1.2), None)
        if old is None or t_now - old[0] < 0.6:
            return False
        _, cur0, lux0 = old
        ind = self.g('indoor_current')
        if cur0 < 3 * ind or (cur - ind) > 0.45 * (cur0 - ind):
            return False                       # panel did not drop sharply
        was_lit = [i for i in range(4) if lux0[i] > 2 * self.g('lit_lux')]
        if not was_lit:
            return True                        # panel collapsed within a second
        ratios = [(lux[i] - INDOOR_LUX) / (lux0[i] - INDOOR_LUX) for i in was_lit]
        return max(ratios) < 0.45 and min(ratios) > 0.02 and max(ratios) / min(ratios) < 3.0

    # ================================================================ LEARN
    def learn_gen(self):
        samples = []
        self.get_logger().warn(
            'LEARN: the day clock is FROZEN (sun_sensor speed 0) during each survey, so the '
            'sun does not move while the robot searches; the clock jumps between surveys. '
            'This is time compression for learning, not a real-time day.')
        t_learn0 = self.now()
        for hour in self.g('learn_hours'):
            fut = self.set_sun(speed=0.0, start_hour=hour)
            while not fut.done():
                yield
            while self.hours is None or abs(self.hours - hour) > 1e-3:
                yield
            yield from self.wait(0.5)
            az, el = self.angles
            sun = (math.sin(math.radians(az)) * math.cos(math.radians(el)),
                   math.cos(math.radians(az)) * math.cos(math.radians(el)))
            lit = [w for w in self.windows.values() if el > 0
                   and sun[0] * math.cos(w['facing_yaw']) + sun[1] * math.sin(w['facing_yaw'])
                   > 0.05]
            x, y, _ = self.pose()
            lit.sort(key=lambda w: math.hypot(w['parking']['x'] - x, w['parking']['y'] - y))
            self.get_logger().info(f'LEARN {hhmm(hour)}: clock frozen, sun az {az:.0f} '
                                   f'el {el:.0f}, sunlit windows: '
                                   f'{", ".join(w["name"] for w in lit) or "none"}')
            entry = {'hour': float(hour), 'azimuth': round(az, 1), 'elevation': round(el, 1),
                     'full_sun_current': round(self.full_sun() + self.g('indoor_current'), 3),
                     'windows': {}}
            t_hour = self.now()
            for w in lit:
                self.detail = f'{hhmm(hour)} {w["name"]}'
                res = yield from self.search_gen(w, az, el)
                entry['windows'][w['name']] = res
                self.get_logger().info(
                    f'LEARN {hhmm(hour)} {w["name"]}: '
                    + (f'best {res["best_current"]:.3f} A at ({res["x"]:+.2f}, {res["y"]:+.2f}), '
                       f'{"reachable" if res["reachable"] else "NOT reachable (edge strip)"}'
                       if res['found'] else 'no sun found')
                    + f', search {res["search_s"]:.0f} sim s')
            entry['survey_sim_s'] = round(self.now() - t_hour, 1)
            samples.append(entry)
            self.save_learned(samples, self.now() - t_learn0, partial=True)
        self.save_learned(samples, self.now() - t_learn0, partial=False)
        self.get_logger().info(f'LEARN done in {self.now() - t_learn0:.0f} sim s; '
                               f'saved {self.g("learned_file")}')
        fut = self.set_sun(speed=1.0)          # unfreeze the clock again
        while not fut.done():
            yield
        self.learned = {'samples': samples}
        self.set_state('NIGHT', self.night_gen(), 'learning finished')

    def save_learned(self, samples, sim_s, partial):
        doc = {
            'learned_on': datetime.now().strftime('%Y-%m-%d %H:%M'),
            'method': ('sun_brain LEARN: day clock frozen (sun_sensor speed 0) during each '
                       'survey, so every entry is the sun at exactly that hour; the robot '
                       'searched each sunlit window with sweep legs + edge-lux gradient climb'),
            'learning_sim_s': round(sim_s, 1),
            'complete': not partial,
            'samples': samples,
        }
        with open(self.g('learned_file'), 'w') as f:
            f.write('# Learned sun spots (written by sun_brain mode:=learn). Poses are in the '
                    'map frame (AMCL), currents in A.\n')
            yaml.safe_dump(doc, f, sort_keys=False)

    def window_frame(self, w):
        f = w['facing_yaw']
        n_out = (math.cos(f), math.sin(f))
        inward = (-n_out[0], -n_out[1])
        along = (-n_out[1], n_out[0])
        half_t = self.cfg.get('wall_thickness', 0.2) / 2
        cx, cy = w['centre']
        return (cx - n_out[0] * half_t, cy - n_out[1] * half_t), inward, along

    def search_points(self, w, az, el):
        """Sweep-leg vertices (world x, y) for one window, ordered by the sun's angle."""
        c, inward, along = self.window_frame(w)
        s_h = (math.sin(math.radians(az)), math.cos(math.radians(az)))
        s_in = -(s_h[0] * inward[0] + s_h[1] * inward[1])     # towards the wall's outside
        s_al = s_h[0] * along[0] + s_h[1] * along[1]
        side = -1.0 if s_al > 0 else 1.0                       # patch shifts away from the sun
        # lateral room available along this wall (stay 1 m from the side walls)
        span = (self.room['y_max'] - self.room['y_min'] if abs(along[1]) > 0.5
                else self.room['x_max'] - self.room['x_min'])
        o_max = span / 2 - 1.0
        d_max = self.g('search_inward')
        d0 = 1.2
        if abs(s_al) > abs(s_in):            # grazing sun: patch slides along the wall
            legs = [(d0, 0.0), (d0, side * o_max), (2.4, side * o_max), (2.4, side * 1.5),
                    (d_max, side * 1.5)]
        else:                                # sun facing the wall: patch goes deep
            legs = [(d0, side * 1.5), (d_max, side * 1.5), (d_max, 0.0), (d0, 0.0),
                    (d0, side * o_max)]
        pts = []
        for d, o in legs:
            x = c[0] + inward[0] * d + along[0] * o
            y = c[1] + inward[1] * d + along[1] * o
            if pts:                                   # pull back towards the previous vertex
                px, py = pts[-1]
                for k in range(21):
                    if self.clear_at(x, y) >= 0.75:
                        break
                    x, y = x + (px - x) * 0.1, y + (py - y) * 0.1
            far_enough = not pts or math.hypot(x - pts[-1][0], y - pts[-1][1]) > 0.3
            if self.clear_at(x, y) >= 0.75 and far_enough:
                pts.append((x, y))
        return pts

    def in_region(self, w, x, y):
        """Is (x, y) in front of this window, within the searched area?"""
        c, inward, along = self.window_frame(w)
        d = (x - c[0]) * inward[0] + (y - c[1]) * inward[1]
        o = (x - c[0]) * along[0] + (y - c[1]) * along[1]
        span = (self.room['y_max'] - self.room['y_min'] if abs(along[1]) > 0.5
                else self.room['x_max'] - self.room['x_min'])
        return 0.0 <= d <= self.g('search_inward') + 1.0 and abs(o) <= span / 2

    def search_gen(self, w, az, el):
        t0 = self.now()
        t_area = None                          # search time counts from reaching the window
        pts = self.search_points(w, az, el)
        result = {'found': False, 'best_current': round(self.current, 3), 'reachable': False,
                  'x': None, 'y': None, 'yaw': None, 'search_s': 0.0}
        for i, (x, y) in enumerate(pts):
            nx, ny = pts[i + 1] if i + 1 < len(pts) else (x, y)
            px, py, pyaw = self.pose()
            yaw = math.atan2(ny - y, nx - x) if i + 1 < len(pts) else math.atan2(y - py, x - px)
            for attempt in range(2):                  # retry a failed leg once
                self.nav_go(x, y, yaw)
                while self.nav_busy():
                    yield
                    px, py, _ = self.pose()
                    if any(self.lit()) and self.in_region(w, px, py):
                        self.nav_cancel()
                        self.cmd()
                        best = yield from self.climb_gen()
                        result.update(best)
                        result['found'] = True
                        result['reachable'] = (best['best_current'] - self.g('indoor_current')
                                               >= 0.25 * self.full_sun())
                        result['search_s'] = round(self.now() - t0, 1)
                        return result
                    if t_area is None and self.in_region(w, px, py):
                        t_area = self.now()
                    if (t_area is not None and self.now() - t_area > self.g('search_max_s')) \
                            or self.now() - t0 > self.g('search_max_s') + 150.0:
                        self.nav_cancel()
                        result['search_s'] = round(self.now() - t0, 1)
                        return result
                if self.nav_status == 'succeeded':
                    break
        result['search_s'] = round(self.now() - t0, 1)
        return result

    def climb_gen(self):
        """Creep up the gradient; return the best (smoothed) current and the pose there."""
        t0 = self.now()
        best_i, best_pose, t_best = -1.0, None, t0
        window = deque(maxlen=5)
        good = 0.9 * self.full_sun() + self.g('indoor_current')
        while self.now() - t0 < self.g('climb_max_s'):
            state = self.creep_step()
            window.append(self.current)
            avg = sum(window) / len(window)
            if avg > best_i * 1.02:
                best_i, best_pose, t_best = avg, self.pose(), self.now()
            if state in ('inside', 'blocked') or avg >= good:
                break
            if self.now() - t_best > 8.0:
                break                                   # no improvement for 8 sim s
            yield
        self.cmd()
        for _ in self.wait(1.5):                       # measure where we stopped
            window.append(self.current)
            yield
        avg = sum(list(window)[-5:]) / min(5, len(window))
        if avg >= 0.95 * best_i:
            best_i, best_pose = max(best_i, avg), self.pose()
        x, y, yaw = best_pose
        return {'best_current': round(best_i, 3), 'x': round(x, 2), 'y': round(y, 2),
                'yaw': round(yaw, 2)}

    # ================================================================ CHOOSE
    def expected_at(self, hour):
        """[(expected current, window, (x, y, yaw))] at a day-hour, from the learned table."""
        samples = sorted(self.learned['samples'], key=lambda s: s['hour'])
        out = []
        for name in self.windows:
            pts = [(s['hour'], s['windows'][name]) for s in samples
                   if name in s['windows'] and s['windows'][name].get('found')
                   and s['windows'][name].get('reachable')]
            if not pts:
                continue
            before = [p for p in pts if p[0] <= hour]
            after = [p for p in pts if p[0] >= hour]
            cand = None
            if before and after:
                (h0, a), (h1, b) = before[-1], after[0]
                if h1 - h0 <= 2.0 + 1e-6:
                    k = 0.0 if h1 == h0 else (hour - h0) / (h1 - h0)
                    cand = (a['best_current'] + k * (b['best_current'] - a['best_current']),
                            (a['x'] + k * (b['x'] - a['x']), a['y'] + k * (b['y'] - a['y']),
                             a['yaw'] + k * wrap(b['yaw'] - a['yaw'])),
                            [(a['x'], a['y']), (b['x'], b['y'])])
            if cand is None:                   # nearest learned hour, if within 1 h, decayed
                h, e = min(pts, key=lambda p: abs(p[0] - hour))
                if abs(h - hour) <= 1.0:
                    cand = (e['best_current'] * (1.0 - 0.5 * abs(h - hour)),
                            (e['x'], e['y'], e['yaw']), [(e['x'], e['y'])])
            if cand:
                out.append((cand[0], name, cand[1], cand[2]))
        return sorted(out, key=lambda c: c[0], reverse=True)

    def choose_gen(self):
        self.cmd()
        yield                                  # (a generator: runs from the next tick on)
        while True:
            if self.elevation() <= 0.5 and self.hours > 12:
                self.set_state('NIGHT', self.night_gen())
                return
            x, y, _ = self.pose()
            rate = self.day_rate()
            best = None
            for cur, name, pose, _seg in self.expected_at(self.hours):
                if self.blacklist.get(name, -1) > self.hours:
                    continue
                travel_h = math.hypot(pose[0] - x, pose[1] - y) / 0.18 * rate
                arr = self.expected_at(self.hours + travel_h)
                hit = next((c for c in arr if c[1] == name), None)
                if hit and hit[0] > 2.5 * self.g('indoor_current') and (best is None
                                                                        or hit[0] > best[0]):
                    best = hit
            if best:
                self.target = {'window': best[1], 'pose': best[2], 'expected': best[0],
                               'segment': best[3]}
                self.set_state('GO', self.go_gen(), f'{best[1]} spot, expect {best[0]:.2f} A')
                return
            self.set_state('WAIT', self.wait_choose_gen(), 'no reachable sun now')
            return

    def wait_choose_gen(self):
        self.cmd()
        yield
        if self.hours - self.last_probe_h > 0.25 and self.elevation() > 2.0:
            # the table has nothing reachable now, but sun may still be right next to us
            after = ('WAIT', self.wait_after_probe_gen(), 'looked around: no sun close by')
            self.set_state('WAIT', self.probe_gen(then=after), 'looking around for sun close by')
            return
        yield from self.wait_after_probe_gen()

    def wait_after_probe_gen(self):
        self.cmd()
        h0 = self.hours
        while self.hours < h0 + 10 / 60:
            if any(self.lit()) and self.hours > self.strip_until:   # not the strip we just left
                self.set_state('SETTLE', self.settle_gen(), 'sun found while waiting')
                return
            yield
        self.set_state('CHOOSE', self.choose_gen())

    # ================================================================ GO / SETTLE / TRACK / LOST
    def go_gen(self):
        x, y, yaw = self.target['pose']
        self.nav_go(x, y, yaw)
        t0 = self.now()
        while self.nav_busy():
            if self.now() - t0 > 180.0:              # Nav2 stuck: give up on this spot
                self.nav_cancel()
                break
            if self.elevation() <= 0.5 and self.hours > 12:
                self.nav_cancel()
                self.set_state('NIGHT', self.night_gen())
                return
            px, py, _ = self.pose()
            if any(self.lit()) and math.hypot(px - x, py - y) < 2.0:
                self.nav_cancel()                    # the patch is already here
                self.set_state('SETTLE', self.settle_gen(), 'sun seen near the target')
                return
            yield
        if self.nav_status == 'succeeded' or any(self.lit()):
            self.set_state('SETTLE', self.settle_gen(), self.target['window'])
        else:
            self.blacklist[self.target['window']] = self.hours + 0.5
            self.set_state('CHOOSE', self.choose_gen(), 'Nav2 failed, skip that spot 30 min')

    def settle_gen(self, lost_state=None):
        t0 = self.now()
        if lost_state == 'EXPLORE' and not any(self.lit()):
            # greedy: we usually drove just past the patch; back up to find it again
            t_back = self.now()
            while self.now() - t_back < 5.0 and not any(self.lit()):
                if not self.safe_cmd(-0.1, 0.0):
                    break
                yield
            self.cmd()
        while True:
            if self.cloud_event():
                self.set_state('LOST', self.lost_gen(lost_state), 'cloud')
                return
            target = self.g('settle_fraction') * self.full_sun() + self.g('indoor_current')
            if self.current >= target:
                self.cmd()
                self.set_state('TRACK', self.track_gen(lost_state),
                               f'{self.current:.2f} A >= {target:.2f} A')
                return
            state = self.creep_step()
            if state == 'blocked':
                self.cmd()
                if self.current > 2.5 * self.g('indoor_current'):
                    self.set_state('TRACK', self.track_gen(lost_state),
                                   'as far into the sun as the walls allow')
                else:
                    if self.target:
                        self.blacklist[self.target['window']] = self.hours + 0.3
                    self.strip_until = self.hours + 0.3
                    self.patch_gone(lost_state, 'sun only against the wall: unreachable')
                return
            if state == 'inside' and self.now() - t0 > 2.0:
                self.set_state('TRACK', self.track_gen(lost_state), 'all edges in the sun')
                return
            if state == 'none' and lost_state is None and self.target \
                    and not self.target.get('probed'):
                self.target['probed'] = True
                self.set_state('SETTLE', self.probe_gen(), 'no sun at the spot: look around')
                return
            if state == 'none' and lost_state is None and self.target \
                    and not self.target.get('swept'):
                self.target['swept'] = True
                self.set_state('SETTLE', self.sweep_gen(), 'no sun at the spot: sweep the '
                               'learned path of the patch')
                return
            if state == 'none' or self.now() - t0 > self.g('settle_max_s'):
                self.cmd()
                if self.current > 2.5 * self.g('indoor_current'):
                    self.set_state('TRACK', self.track_gen(lost_state), 'partial sun')
                else:
                    if self.target:
                        self.blacklist[self.target['window']] = self.hours + 0.3
                    self.patch_gone(lost_state, 'no sun here')
                return
            yield

    def probe_gen(self, then=None):
        """Look around for sun close by: one turn on the spot (the edge sensors sweep a circle),
        then 0.4 m forward/back, a quarter turn, 0.4 m forward/back. Stops at the first sun."""
        self.last_probe_h = self.hours

        def found():
            return any(self.lit()) and self.hours > self.strip_until

        def turn(angle):
            t_end = self.now() + abs(angle) / 0.5
            while self.now() < t_end:
                if found():
                    return True
                if not self.safe_cmd(0.0, math.copysign(0.5, angle)):
                    break
                yield
            self.cmd()
            return found()

        def drive(dist):
            t_end = self.now() + abs(dist) / 0.1
            while self.now() < t_end:
                if found():
                    return True
                if not self.safe_cmd(math.copysign(0.1, dist), 0.0):
                    break
                yield
            self.cmd()
            return found()

        for step in (lambda: turn(2 * math.pi), lambda: drive(0.4), lambda: drive(-0.8),
                     lambda: drive(0.4), lambda: turn(math.pi / 2), lambda: drive(0.4),
                     lambda: drive(-0.8), lambda: drive(0.4)):
            if (yield from step()):
                self.cmd()
                self.set_state('SETTLE', self.settle_gen(), 'found sun while looking around')
                return
        self.cmd()
        if then is not None:
            self.set_state(then[0], then[1], then[2])
        elif self.target and not self.target.get('swept'):
            self.target['swept'] = True
            self.set_state('SETTLE', self.sweep_gen(), 'nothing close by: sweep the learned '
                           'path of the patch')
        else:
            self.set_state('CHOOSE', self.choose_gen(), 'nothing close by')

    def sweep_gen(self):
        """Drive along the patch's learned path (earlier spot -> later spot) looking for sun."""
        pts = list(self.target.get('segment', []))
        x0, y0, _ = self.pose()
        pts.sort(key=lambda p: math.hypot(p[0] - x0, p[1] - y0))   # nearer end first
        for (x, y) in pts:
            self.nav_go(x, y, math.atan2(y - y0, x - x0))
            while self.nav_busy():
                if any(self.lit()):
                    self.nav_cancel()
                    self.cmd()
                    self.set_state('SETTLE', self.settle_gen(), 'found the patch on its path')
                    return
                yield
            x0, y0 = x, y
        self.blacklist[self.target['window']] = self.hours + 0.3
        self.set_state('CHOOSE', self.choose_gen(), 'patch not found, skip that window 20 min')

    def patch_gone(self, lost_state, why):
        if lost_state == 'EXPLORE':
            self.set_state('EXPLORE', self.explore_gen(), why)
        else:
            self.set_state('CHOOSE', self.choose_gen(), why)

    def track_gen(self, lost_state=None):
        self.cmd()
        ref = self.current                       # panel current when tracking started
        ref_full = max(self.full_sun(), 1e-3)
        next_check = self.now() + self.g('track_period')
        while True:
            if self.elevation() <= 0.5 and self.hours > 12:
                self.set_state('NIGHT', self.night_gen())
                return
            if self.cloud_event():
                self.set_state('LOST', self.lost_gen(lost_state), 'cloud')
                return
            if self.now() >= next_check:
                next_check = self.now() + self.g('track_period')
                lit = self.lit()
                ind = self.g('indoor_current')
                if not any(lit) and self.current < 2.5 * ind:
                    self.patch_gone(lost_state, 'patch moved away')
                    return
                expected_ref = ref * self.full_sun() / ref_full   # ref scaled to the elevation
                if any(lit) and self.current < 0.6 * expected_ref:
                    self.set_state('SETTLE', self.settle_gen(lost_state), 'panel dropped')
                    return
                if any(lit) and not all(lit):
                    t_end = self.now() + 1.5           # short nudge towards the bright edge
                    while self.now() < t_end and self.creep_step() == 'moving':  # not blocked
                        if self.cloud_event():
                            self.set_state('LOST', self.lost_gen(lost_state), 'cloud')
                            return
                        yield
                    self.cmd()
                    ref = max(ref, self.current)
            yield

    def lost_gen(self, lost_state=None):
        self.cmd()
        pre = max(h[1] for h in self.hist)       # current just before the drop
        t0 = self.now()
        t_dark = None
        cloud_level = math.inf                   # lowest current seen under the cloud
        self.detail = f'cloud? holding up to {self.g("cloud_hold"):.0f} s'
        while self.now() - t0 < self.g('cloud_hold'):
            # under a cloud the patch is still seen, dimmed to ~10 % (thousands of lux, far above
            # indoor light); if every sensor falls to indoor level the patch itself has moved off
            if any(self.lit()):
                t_dark = None
            elif t_dark is None:
                t_dark = self.now()
            elif self.now() - t_dark > 3.0:
                self.patch_gone(lost_state, 'patch moved away during the cloud')
                return
            if self.now() - t0 > 2.0:
                cloud_level = min(cloud_level, self.current)
            if self.current >= 0.6 * pre:
                self.set_state('TRACK', self.track_gen(lost_state),
                               f'sun back after {self.now() - t0:.0f} s')
                return
            ind = self.g('indoor_current')
            if self.now() - t0 > 2.0 and self.current - ind > 4.0 * (cloud_level - ind) + 0.01:
                # cloud over, but the patch slid partly away meanwhile: creep back into it
                self.set_state('SETTLE', self.settle_gen(lost_state),
                               f'cloud over after {self.now() - t0:.0f} s, patch moved: re-settle')
                return
            if self.elevation() <= 0.5 and self.hours > 12:
                self.set_state('NIGHT', self.night_gen())
                return
            yield
        self.patch_gone(lost_state, f'no sun for {self.g("cloud_hold"):.0f} s')

    # ================================================================ NIGHT
    def night_gen(self):
        self.cmd()
        spot = None
        if self.learned:
            for s in sorted(self.learned['samples'], key=lambda s: s['hour']):
                ok = [(e['best_current'], e) for e in s['windows'].values()
                      if e.get('found') and e.get('reachable')]
                if ok:
                    spot = max(ok, key=lambda c: c[0])[1]
                    break
        if spot:
            self.target = {'window': 'morning', 'pose': (spot['x'], spot['y'], spot['yaw']),
                           'expected': 0.0}
            self.detail = 'parking at the first morning spot'
            self.nav_go(spot['x'], spot['y'], spot['yaw'])
            while self.nav_busy():
                yield
        self.detail = 'parked for the night'
        self.cmd()
        while True:
            yield

    # ================================================================ baselines
    def parked_gen(self):
        p = self.windows[self.g('park_window')]['parking']
        self.target = {'window': self.g('park_window'), 'pose': (p['x'], p['y'], p['yaw']),
                       'expected': 0.0}
        self.nav_go(p['x'], p['y'], p['yaw'])
        while self.nav_busy():
            yield
        self.set_state('PARKED', self.idle_stop_gen(), f'{self.g("park_window")} spot')

    def idle_stop_gen(self):
        self.cmd()
        while True:
            yield

    def explore_gen(self):
        """Greedy baseline: wander (lidar obstacle avoidance) until an edge sensor sees sun."""
        turn_dir = 1.0
        while True:
            if self.elevation() <= 0.5 and self.hours > 12:
                self.cmd()
                self.set_state('NIGHT', self.idle_stop_gen())
                return
            if any(self.lit()):
                self.cmd()
                self.set_state('SETTLE', self.settle_gen('EXPLORE'), 'edge sensor sees sun')
                return
            if self.scan is not None:
                s = self.scan

                def sector(lo, hi):
                    v = [r if math.isfinite(r) else s.range_max
                         for i, r in enumerate(s.ranges)
                         if lo <= math.degrees(s.angle_min + i * s.angle_increment) <= hi]
                    return min(v) if v else s.range_max
                front, left, right = sector(-35, 35), sector(35, 100), sector(-100, -35)
                if front > 1.0:
                    self.cmd(0.2, 0.4 * (left - right) / max(left + right, 1e-3))
                else:
                    if front > 0.7:
                        turn_dir = 1.0 if left >= right else -1.0
                    self.cmd(0.0, 0.6 * turn_dir)
            yield


def main():
    # keep the ROS context alive on Ctrl+C so the final stop command still gets published
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = SunBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.nav_cancel()
            node.cmd()
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
