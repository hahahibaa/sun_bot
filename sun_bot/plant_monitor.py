"""Simulated soil moisture of the potted plant on sun_bot.

    ros2 run sun_bot plant_monitor
    ros2 run sun_bot plant_monitor --ros-args -p dry_scale:=20.0      # dry 20x faster (testing)
    ros2 service call /plant/water std_srvs/srv/Trigger                 # refill to 70 %

Moisture starts at 60 % and dries with the day clock (/sun/clock_hours), faster while the
robot is in the sun (/sun/panel_current above sun_current). Honks below 30 % and keeps honking
until moisture is back above 45 % (hysteresis). A honk is a log line, a red "THIRSTY!" marker
above the robot in RViz and, only with -p beep:=true, a beep through libpulse-simple (WSLg).

Publishes /plant/moisture (std_msgs/Float32, %), /plant/thirsty (std_msgs/Bool),
/plant/marker (visualization_msgs/Marker).
"""
import ctypes
import ctypes.util
import math
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Float64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class _SampleSpec(ctypes.Structure):
    _fields_ = [('format', ctypes.c_int), ('rate', ctypes.c_uint32), ('channels', ctypes.c_uint8)]


def make_beeper(freq=880.0, seconds=0.25, rate=22050):
    """Return a function that plays a short beep via libpulse-simple (WSLg), or None."""
    name = ctypes.util.find_library('pulse-simple')
    if not name:
        return None
    try:
        lib = ctypes.CDLL(name)
    except OSError:
        return None
    lib.pa_simple_new.restype = ctypes.c_void_p
    lib.pa_simple_new.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int,
                                  ctypes.c_char_p, ctypes.c_char_p,
                                  ctypes.POINTER(_SampleSpec), ctypes.c_void_p,
                                  ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    lib.pa_simple_write.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t,
                                    ctypes.POINTER(ctypes.c_int)]
    lib.pa_simple_drain.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
    lib.pa_simple_free.argtypes = [ctypes.c_void_p]
    n = int(seconds * rate)
    pcm = b''.join(struct.pack('<h', int(12000 * math.sin(2 * math.pi * freq * i / rate)
                                         * min(1.0, (n - i) / (0.02 * rate))))
                   for i in range(n))
    spec = _SampleSpec(3, rate, 1)                  # PA_SAMPLE_S16LE, mono

    def play():
        err = ctypes.c_int(0)
        h = lib.pa_simple_new(None, b'sun_bot', 1, None, b'thirsty plant', ctypes.byref(spec),
                              None, None, ctypes.byref(err))   # 1 = PA_STREAM_PLAYBACK
        if not h:
            return False
        lib.pa_simple_write(h, pcm, len(pcm), ctypes.byref(err))
        lib.pa_simple_drain(h, ctypes.byref(err))
        lib.pa_simple_free(h)
        return True

    return play


class PlantMonitor(Node):

    def __init__(self):
        super().__init__('plant_monitor', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        p = self.declare_parameter
        p('start_moisture', 60.0)       # %
        p('water_to', 70.0)             # % after /plant/water
        p('dry_rate', 1.5)              # % per day-hour, always
        p('sun_dry_rate', 3.0)          # % per day-hour extra while in the sun
        p('dry_scale', 1.0)             # multiply both rates (testing)
        p('sun_current', 0.05)          # A: panel above this = plant is in the sun
        p('thirsty_below', 30.0)        # % start honking
        p('ok_above', 45.0)             # % stop honking
        p('honk_period', 5.0)           # sim s between honks while thirsty
        p('beep', False)                # play a real sound per honk (opt-in: it is loud)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.moisture = g('start_moisture')
        self.thirsty = False
        self.last_hours = None
        self.in_sun = False
        self.last_honk = -math.inf
        self.beeper = make_beeper() if g('beep') else None

        self.pub_m = self.create_publisher(Float32, '/plant/moisture', 10)
        self.pub_t = self.create_publisher(Bool, '/plant/thirsty', 10)
        self.pub_marker = self.create_publisher(Marker, '/plant/marker', 10)
        self.create_subscription(Float64, '/sun/clock_hours', self.on_hours, 10)
        self.create_subscription(Float32, '/sun/panel_current',
                                 lambda m: setattr(self, 'in_sun',
                                                   m.data > g('sun_current')), 10)
        self.create_service(Trigger, '/plant/water', self.on_water)
        self.create_timer(0.5, self.publish)
        self.get_logger().info(
            f'plant at {self.moisture:.0f} %, honks below {g("thirsty_below"):.0f} %, '
            f'quiet again above {g("ok_above"):.0f} %')

    def on_hours(self, msg):
        h = msg.data
        if self.last_hours is not None and 0.0 < h - self.last_hours < 1.0:
            rate = self.g('dry_rate') + (self.g('sun_dry_rate') if self.in_sun else 0.0)
            self.moisture = max(0.0, self.moisture - rate * self.g('dry_scale')
                                * (h - self.last_hours))
        self.last_hours = h

    def on_water(self, request, response):
        before = self.moisture
        self.moisture = self.g('water_to')
        response.success = True
        response.message = f'watered: {before:.1f} % -> {self.moisture:.1f} %'
        self.get_logger().info(response.message)
        return response

    def publish(self):
        if not self.thirsty and self.moisture < self.g('thirsty_below'):
            self.thirsty = True
            self.get_logger().warn(f'THIRSTY! moisture {self.moisture:.1f} % '
                                   f'(< {self.g("thirsty_below"):.0f} %), honking')
        elif self.thirsty and self.moisture > self.g('ok_above'):
            self.thirsty = False
            self.get_logger().info(f'happy again: moisture {self.moisture:.1f} % '
                                   f'(> {self.g("ok_above"):.0f} %), honk off')
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.thirsty and now - self.last_honk >= self.g('honk_period'):
            self.last_honk = now
            self.get_logger().warn(f'HONK! plant thirsty, moisture {self.moisture:.1f} %')
            if self.beeper:
                threading.Thread(target=self.beeper, daemon=True).start()
        self.pub_m.publish(Float32(data=float(self.moisture)))
        self.pub_t.publish(Bool(data=self.thirsty))

        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns, m.id, m.type = 'plant', 0, Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD if self.thirsty else Marker.DELETE
        m.pose.position.z = 1.4
        m.pose.orientation.w = 1.0
        m.scale.z = 0.5
        m.color.r, m.color.a = 1.0, 1.0
        m.text = 'THIRSTY!'
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = PlantMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
