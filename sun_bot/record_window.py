"""Record one X11 window (e.g. RViz under WSLg) to an H.264 MP4, without ffmpeg.

    ros2 run sun_bot record_window --title 'demo.rviz' --out ~/sun_ws/docs/demo_rviz.mp4
    ros2 run sun_bot record_window --title Gazebo --out gazebo.mp4 --fps 10 --seconds 60

Grabs the window with XGetImage (works on WSLg, where grabbing the root window fails) and writes
with OpenCV's built-in FFmpeg encoder. Runs until --seconds have passed, the window closes, or
Ctrl+C. Frames are paced to real time (a slow grab repeats the previous frame).
"""
import argparse
import ctypes
import ctypes.util
import os
import re
import signal
import subprocess
import time

import cv2
import numpy as np


class XImage(ctypes.Structure):
    _fields_ = [('width', ctypes.c_int), ('height', ctypes.c_int), ('xoffset', ctypes.c_int),
                ('format', ctypes.c_int), ('data', ctypes.c_void_p),
                ('byte_order', ctypes.c_int), ('bitmap_unit', ctypes.c_int),
                ('bitmap_bit_order', ctypes.c_int), ('bitmap_pad', ctypes.c_int),
                ('depth', ctypes.c_int), ('bytes_per_line', ctypes.c_int),
                ('bits_per_pixel', ctypes.c_int)]


def find_window(title):
    tree = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
    best = None
    for line in tree.splitlines():
        m = re.match(r'\s*(0x[0-9a-f]+) "([^"]*)".*?\s(\d+)x(\d+)\+', line)
        if m and re.search(title, m.group(2)) and int(m.group(3)) > 200:
            area = int(m.group(3)) * int(m.group(4))
            if best is None or area > best[1]:
                best = (int(m.group(1), 16), area, m.group(2))
    return best


class Grabber:
    def __init__(self, window):
        self.x = ctypes.CDLL(ctypes.util.find_library('X11'))
        self.x.XOpenDisplay.restype = ctypes.c_void_p
        self.x.XGetImage.restype = ctypes.POINTER(XImage)
        self.x.XGetImage.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
                                     ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_int]
        self.x.XGetGeometry.argtypes = [ctypes.c_void_p, ctypes.c_ulong] + \
            [ctypes.c_void_p] + [ctypes.POINTER(ctypes.c_int)] * 2 + \
            [ctypes.POINTER(ctypes.c_uint)] * 4
        self.x.XDestroyImage.argtypes = [ctypes.POINTER(XImage)]
        self.d = self.x.XOpenDisplay(None)
        self.w = window

    def size(self):
        root = ctypes.c_ulong()
        x, y = ctypes.c_int(), ctypes.c_int()
        w, h, b, d = (ctypes.c_uint() for _ in range(4))
        if not self.x.XGetGeometry(self.d, self.w, ctypes.byref(root), ctypes.byref(x),
                                   ctypes.byref(y), ctypes.byref(w), ctypes.byref(h),
                                   ctypes.byref(b), ctypes.byref(d)):
            return None
        return w.value, h.value

    def grab(self):
        sz = self.size()
        if sz is None:
            return None
        w, h = sz
        img = self.x.XGetImage(self.d, self.w, 0, 0, w, h, 0xFFFFFFFF, 2)   # ZPixmap
        if not img:
            return None
        im = img.contents
        buf = ctypes.string_at(im.data, im.bytes_per_line * im.height)
        arr = np.frombuffer(buf, np.uint8).reshape(im.height, im.bytes_per_line // 4, 4)
        frame = arr[:, :im.width, :3].copy()                                   # BGRA -> BGR
        self.x.XDestroyImage(img)
        return frame


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--title', default='RViz', help='regex matched against window titles')
    ap.add_argument('--out', default=os.path.expanduser('~/sun_ws/docs/demo_rviz.mp4'))
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--seconds', type=float, default=0.0, help='0 = until Ctrl+C / window closes')
    ap.add_argument('--width', type=int, default=1920, help='output width (keeps aspect)')
    ap.add_argument('--speedup', type=float, default=1.0,
                    help='time-lapse: 5 = grab every 5/fps s, play at fps (5x faster)')
    a = ap.parse_args()

    found = None
    for _ in range(120):
        found = find_window(a.title)
        if found:
            break
        time.sleep(1.0)
    if not found:
        raise SystemExit(f'no window title matching {a.title!r}')
    wid, _, name = found
    g = Grabber(wid)
    first = g.grab()
    if first is None:
        raise SystemExit(f'cannot grab window {name!r}')
    h0, w0 = first.shape[:2]
    out_w = min(a.width, w0) // 2 * 2
    out_h = int(round(h0 * out_w / w0)) // 2 * 2
    writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'avc1'), a.fps, (out_w, out_h))
    if not writer.isOpened():
        writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (out_w, out_h))
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    print(f'recording "{name}" ({w0}x{h0}) -> {a.out} at {out_w}x{out_h}, {a.fps:g} fps'
          + (f', {a.speedup:g}x time-lapse' if a.speedup != 1 else ''), flush=True)
    t0 = time.monotonic()
    n = 0
    last = cv2.resize(first, (out_w, out_h), interpolation=cv2.INTER_AREA)
    grab_ms = []
    while not stop and (a.seconds <= 0 or time.monotonic() - t0 < a.seconds):
        t = time.monotonic()
        frame = g.grab()
        if frame is None:
            break                                    # window closed
        grab_ms.append((time.monotonic() - t) * 1000)
        if frame.shape[:2] != (h0, w0):
            frame = cv2.resize(frame, (w0, h0))
        last = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)
        due = int((time.monotonic() - t0) * a.fps / a.speedup) + 1
        while n < due:                               # real-time pacing
            writer.write(last)
            n += 1
        time.sleep(max(0.0, t0 + n * a.speedup / a.fps - time.monotonic()))
    writer.release()
    print(f'saved {a.out}: {n} frames, {n / a.fps:.1f} s of video, '
          f'grab {np.mean(grab_ms):.0f} ms/frame (achieved ~'
          f'{min(a.fps, 1000 / max(np.mean(grab_ms), 1)):.0f} fps)', flush=True)


if __name__ == '__main__':
    main()
