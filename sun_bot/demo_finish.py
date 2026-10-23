"""Post-process the recorded demo video: add a caption bar (title, time-lapse note, state legend)
over RViz's status bar, optionally cut it to a maximum length.

    python3 -m sun_bot.demo_finish ~/sun_ws/docs/demo_rviz_raw.mp4 ~/sun_ws/docs/demo_rviz.mp4
"""
import argparse

import cv2

STATES = [('GO', (0.42, 0.68, 0.84)), ('SETTLE', (0.99, 0.68, 0.42)),
          ('TRACK', (0.19, 0.64, 0.33)), ('LOST', (0.39, 0.39, 0.39)),
          ('WAIT', (0.85, 0.85, 0.85)), ('CHOOSE', (0.6, 0.6, 0.8)), ('NIGHT', (0.03, 0.19, 0.42))]


TITLE = ('sun_bot demo - simulated day, 5x time-lapse '
         '(clock at the top is the simulated time of day)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--max-seconds', type=float, default=0.0)
    ap.add_argument('--title', default=TITLE)
    a = ap.parse_args()
    c = cv2.VideoCapture(a.src)
    fps, w, h = c.get(cv2.CAP_PROP_FPS), int(c.get(3)), int(c.get(4))
    out = cv2.VideoWriter(a.dst, cv2.VideoWriter_fourcc(*'avc1'), fps, (w, h))
    n_max = int(a.max_seconds * fps) if a.max_seconds > 0 else 10 ** 9
    n = 0
    bar = 34
    while n < n_max:
        ok, f = c.read()
        if not ok:
            break
        cv2.rectangle(f, (0, h - bar), (w, h), (30, 30, 30), -1)
        cv2.putText(f, a.title, (12, h - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 235, 235), 1,
                    cv2.LINE_AA)
        x = w - 10
        for name, rgb in reversed(STATES):
            tw = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0][0]
            x -= tw + 32
            bgr = tuple(int(255 * v) for v in reversed(rgb))
            cv2.rectangle(f, (x, h - 26), (x + 18, h - 10), bgr, -1)
            cv2.putText(f, name, (x + 24, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(f, 'state strip:', (x - 120, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (235, 235, 235), 1, cv2.LINE_AA)
        out.write(f)
        n += 1
    out.release()
    print(f'saved {a.dst}: {n} frames, {n / fps:.1f} s')


if __name__ == '__main__':
    main()
