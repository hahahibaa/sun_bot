"""Drive to a window's parking spot (from config/windows.yaml) with Nav2.

    ros2 run sun_bot go_to_window east          # one window: east, south or west
    ros2 run sun_bot go_to_window all           # east -> south -> west
    ros2 run sun_bot go_to_window all --ros-args -p windows_file:=/path/to/windows.yaml

Needs nav.launch.py running. For each window it logs success/failure, the time taken (sim and
wall) and the final position and heading error of map -> base_link versus the parking spot,
then prints a summary table. Exit code 0 only if every window was reached.
"""
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

ORDER = ['east', 'south', 'west']
TIMEOUT = 600.0  # sim s per window before the goal is cancelled


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def sim_now(nav):
    return nav.get_clock().now().nanoseconds * 1e-9


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def robot_pose(nav, tf_buffer):
    """map -> base_link as (x, y, yaw), or None."""
    for _ in range(20):
        rclpy.spin_once(nav, timeout_sec=0.1)
        try:
            t = tf_buffer.lookup_transform('map', 'base_link', Time()).transform
            return t.translation.x, t.translation.y, yaw_of(t.rotation)
        except TransformException:
            pass
    return None


def visit(nav, tf_buffer, window):
    name, p = window['name'], window['parking']
    nav.info(f'--- {name}: going to parking spot ({p["x"]:+.2f}, {p["y"]:+.2f}, '
             f'yaw {math.degrees(p["yaw"]):+.0f} deg)')
    t_sim, t_wall = sim_now(nav), time.monotonic()
    if not nav.goToPose(make_pose(nav, p['x'], p['y'], p['yaw'])):
        return {'name': name, 'ok': False, 'status': 'goal rejected'}

    last_log = 0.0
    while not nav.isTaskComplete():
        elapsed = sim_now(nav) - t_sim
        fb = nav.getFeedback()
        if fb and elapsed - last_log >= 10.0:
            last_log = elapsed
            nav.info(f'{name}: {fb.distance_remaining:.2f} m to go, {elapsed:.0f} s sim, '
                     f'{fb.number_of_recoveries} recoveries')
        if elapsed > TIMEOUT:
            nav.warn(f'{name}: no result after {TIMEOUT:.0f} s sim, cancelling')
            nav.cancelTask()

    result = nav.getResult()
    res = {'name': name, 'ok': result == TaskResult.SUCCEEDED,
           'status': {TaskResult.SUCCEEDED: 'succeeded', TaskResult.CANCELED: 'canceled',
                      TaskResult.FAILED: 'failed'}.get(result, str(result)),
           'sim_s': sim_now(nav) - t_sim, 'wall_s': time.monotonic() - t_wall}
    pose = robot_pose(nav, tf_buffer)
    if pose:
        x, y, yaw = pose
        res['pos_err'] = math.hypot(x - p['x'], y - p['y'])
        res['yaw_err'] = math.degrees(wrap(yaw - p['yaw']))
        res['final'] = (x, y, math.degrees(yaw))
    log = nav.info if res['ok'] else nav.error
    log(f'{name}: {res["status"].upper()} in {res["sim_s"]:.1f} s sim ({res["wall_s"]:.1f} s wall)'
        + (f', position error {res["pos_err"]:.3f} m, heading error {res["yaw_err"]:+.1f} deg'
           if pose else ', final pose unknown'))
    return res


def main():
    args = remove_ros_args(sys.argv)[1:]
    if len(args) != 1 or args[0] not in ORDER + ['all']:
        print(__doc__)
        print(f'usage: go_to_window {{{"|".join(ORDER)}|all}}')
        return 2

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    path = nav.declare_parameter('windows_file', os.path.join(
        get_package_share_directory('sun_bot'), 'config', 'windows.yaml')).value
    with open(path) as f:
        windows = {w['name']: w for w in yaml.safe_load(f)['windows']}
    names = ORDER if args[0] == 'all' else args[0:1]
    tf_buffer = Buffer()
    TransformListener(tf_buffer, nav)

    results = []
    try:
        # Not waitUntilNav2Active(localizer='amcl'): that publishes an initial pose of (0, 0)
        # and would teleport AMCL back to the origin if the robot is somewhere else.
        # AMCL sets its own initial pose (set_initial_pose), so just wait for its estimate.
        nav.waitUntilNav2Active(localizer='controller_server')
        while not nav.initial_pose_received:
            nav.info('waiting for amcl_pose ...')
            rclpy.spin_once(nav, timeout_sec=1.0)
        while sim_now(nav) <= 0.0:              # no /clock yet: timings would start from 0
            rclpy.spin_once(nav, timeout_sec=0.1)
        for name in names:
            results.append(visit(nav, tf_buffer, windows[name]))
    except KeyboardInterrupt:
        nav.warn('interrupted, cancelling the current goal')
        nav.cancelTask()

    if results:
        lines = ['', f'{"window":7s} {"reached":8s} {"sim s":>7s} {"wall s":>7s} '
                     f'{"pos err m":>9s} {"yaw err deg":>11s}']
        for r in results:
            lines.append(
                f'{r["name"]:7s} {"yes" if r["ok"] else "NO (" + r["status"] + ")":8s} '
                f'{r.get("sim_s", float("nan")):7.1f} {r.get("wall_s", float("nan")):7.1f} '
                f'{r.get("pos_err", float("nan")):9.3f} {r.get("yaw_err", float("nan")):+11.1f}')
        nav.info('\n'.join(lines))
    ok = bool(results) and all(r['ok'] for r in results) and len(results) == len(names)
    nav.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
