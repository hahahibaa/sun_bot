"""Top-down replay video of a logged sun_bot run (from sun_logger CSV), no simulator needed.

    ros2 run sun_bot sun_replay ~/sun_ws/docs/demo_run.csv --out ~/sun_ws/docs/demo_replay.mp4
    ros2 run sun_bot sun_replay ~/sun_ws/docs/run_learn.csv --learned --seconds 30 \\
        --out ~/sun_ws/docs/learning_replay.mp4

Room (saved map), windows, pallets, the sun patches (recomputed from the logged sun angles, with
pallet shadows), the robot (ground truth) and its trail; captions with time, state, panel
current and plant moisture; a growing panel-current chart. --learned (for a mode:=learn log)
shows the survey in progress and makes each learned spot appear when its search finishes.
"""
import argparse
import csv
import math
import os

from ament_index_python.packages import get_package_share_directory
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

from sun_bot.sun_geometry import (all_patches, body_to_world, load_windows,  # noqa: E402
                                  sun_vector)
from sun_bot.sun_report import STATE_COLOURS, draw_room, hour_colour  # noqa: E402

BODY = [(0.12, 0.15), (0.12, -0.15), (-0.28, -0.15), (-0.28, 0.15)]
WHEELS = [(0.0, 0.18), (0.0, -0.18)]


def hhmm(h):
    return f'{int(h) % 24:02d}:{int(round((h % 1) * 60)) % 60:02d}'


def learned_events(rows, doc):
    """[(sim time when it appears, hour, window, entry)] from the state_detail transitions."""
    out, seen = [], set()
    for i in range(1, len(rows)):
        prev, cur = rows[i - 1]['state_detail'], rows[i]['state_detail']
        if prev != cur and prev.startswith('LEARN ') and len(prev.split()) == 3:
            _, hm, win = prev.split()
            hour = int(hm[:2]) + int(hm[3:]) / 60
            if (hour, win) in seen:
                continue
            seen.add((hour, win))
            smp = next((s for s in doc['samples'] if abs(s['hour'] - hour) < 1e-3), None)
            if smp and win in smp['windows']:
                out.append((float(rows[i]['sim_time']), hour, win, smp['windows'][win]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--out', default=os.path.expanduser('~/sun_ws/docs/demo_replay.mp4'))
    ap.add_argument('--seconds', type=float, default=120.0, help='video length')
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--learned', nargs='?', const=os.path.expanduser(
        '~/sun_ws/src/sun_bot/config/learned_sun.yaml'), help='learning replay (yaml)')
    ap.add_argument('--title', default='')
    ap.add_argument('--windows', default=os.path.join(
        get_package_share_directory('sun_bot'), 'config', 'windows.yaml'))
    a = ap.parse_args()

    with open(a.csv) as f:
        rows = list(csv.DictReader(f))
    cfg, room = load_windows(a.windows)
    events = []
    if a.learned:
        with open(a.learned) as f:
            events = learned_events(rows, yaml.safe_load(f))
    t = np.array([float(r['sim_time']) for r in rows])
    n_frames = int(a.seconds * a.fps)
    idx = np.searchsorted(t, np.linspace(t[0], t[-1], n_frames)).clip(0, len(rows) - 1)

    fig = plt.figure(figsize=(19.2, 10.8), dpi=100)
    ax = fig.add_axes([0.03, 0.06, 0.64, 0.88])
    draw_room(ax, cfg, room)
    ax.set_xlabel('')
    ax.set_ylabel('')
    title = a.title or ('Day 1: LEARN - where does the sun land at each hour?' if a.learned
                        else 'sun_bot: keeping the plant in the sun')
    fig.text(0.03, 0.955, title, fontsize=22, weight='bold')
    info = fig.text(0.70, 0.86, '', fontsize=20, family='monospace', va='top')
    note = fig.text(0.70, 0.60, '', fontsize=15, va='top', color='#444444')
    axc = fig.add_axes([0.71, 0.10, 0.27, 0.36])
    hrs = np.array([float(r['hours']) for r in rows])
    cur = np.array([float(r['panel_current']) for r in rows])
    full = np.array([0.55 * max(0.0, math.sin(math.radians(float(r['elevation'])))) + 0.01
                     for r in rows])
    if a.learned:
        axc.set_visible(False)
    else:
        axc.set_xlim(hrs.min(), max(hrs.max(), hrs.min() + 1))
        axc.set_ylim(0, 0.5)
        axc.set_xlabel('time of day (h)')
        axc.set_ylabel('panel current (A)')
        axc.grid(alpha=0.3)
        axc.plot(hrs, full, color='gray', ls='--', lw=1, label='full sun')
        line_cur, = axc.plot([], [], color='tab:red', lw=1.5, label='panel')
        axc.legend(loc='upper right', fontsize=10)
    trail, = ax.plot([], [], color='tab:blue', lw=1.2, alpha=0.6)
    dyn = []

    writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'avc1'), a.fps, (1920, 1080))
    if not writer.isOpened():
        writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (1920, 1080))
    trail_x, trail_y = [], []
    last_i = 0
    for k, i in enumerate(idx):
        r = rows[i]
        for art in dyn:
            art.remove()
        dyn = []
        az, el = float(r['azimuth']), float(r['elevation'])
        cloudy = r['cloudy'] == '1'
        if el > 0:
            for pieces in all_patches(cfg, room, sun_vector(az, el), 0.0).values():
                for poly in pieces:
                    dyn.append(ax.add_patch(Polygon(
                        poly, closed=True, facecolor='#f5c400', edgecolor='none',
                        alpha=0.25 if cloudy else 0.65)))
        for j in range(last_i, i + 1):
            trail_x.append(float(rows[j]['x']))
            trail_y.append(float(rows[j]['y']))
        last_i = i + 1
        trail.set_data(trail_x, trail_y)
        pose = (float(r['x']), float(r['y']), math.radians(float(r['yaw_deg'])))
        state = r['state'] or 'INIT'
        colour = STATE_COLOURS.get(state, '#888888')
        dyn.append(ax.add_patch(Polygon([body_to_world(pose, *c) for c in BODY], closed=True,
                                        facecolor='#e69a1e', edgecolor='black', lw=1.5)))
        for wx, wy in WHEELS:
            p = body_to_world(pose, wx, wy)
            dyn.append(ax.plot(p[0], p[1], 's', color='black', ms=5)[0])
        tip = body_to_world(pose, 0.45, 0.0)
        dyn.append(ax.annotate('', xy=tip, xytext=pose[:2],
                               arrowprops=dict(arrowstyle='->', color='black', lw=2)))
        plant = body_to_world(pose, -0.19, 0.0)
        dyn.append(ax.plot(*plant, 'o', color='#2ca02c', ms=8)[0])
        dyn.append(ax.add_patch(plt.Circle(pose[:2], 0.55, fill=False, color=colour, lw=3)))
        if r['thirsty'] == '1':
            dyn.append(ax.text(pose[0], pose[1] + 0.9, 'THIRSTY!', color='red', fontsize=18,
                               weight='bold', ha='center'))
        if cloudy:
            dyn.append(ax.text(0, 6.3, 'CLOUD', color='#555555', fontsize=20, weight='bold',
                               ha='center'))
        for ev_t, hour, win, e in events:
            if ev_t <= float(r['sim_time']) and e.get('found'):
                c = hour_colour(hour)
                dyn.append(ax.plot(e['x'], e['y'], 'o', color=c, ms=14, mec='black', mew=1.5)[0])
                below = int(round(hour)) in (9, 13, 17)        # alternate: no overlap
                dyn.append(ax.text(e['x'] + 0.25, e['y'] + (-0.75 if below else 0.25),
                                   f'{hhmm(hour)}\n{e["best_current"]:.2f} A', fontsize=10))
            elif ev_t <= float(r['sim_time']):
                w = next(w for w in cfg['windows'] if w['name'] == win)
                dyn.append(ax.plot(w['parking']['x'], w['parking']['y'], 'x', color='gray',
                                   ms=14, mew=2.5)[0])
        moist = float(r['moisture']) if r['moisture'] not in ('', 'nan') else float('nan')
        info.set_text(f'time   {hhmm(float(r["hours"]))}\n'
                      f'state  {state}\n'
                      f'panel  {float(r["panel_current"]):.2f} A\n'
                      f'sun    az {az:3.0f}  el {el:3.0f}\n'
                      + (f'plant  {moist:.0f} %' if not math.isnan(moist) else ''))
        info.set_color(colour if state not in ('WAIT', 'INIT') else 'black')
        detail = r.get('state_detail', state)[len(state):].strip()
        if a.learned and state == 'LEARN':
            detail = (f'survey {detail}' if detail else 'surveying') + \
                '\n(day clock frozen during each survey)'
        note.set_text(detail[:60])
        if not a.learned:
            line_cur.set_data(hrs[:i + 1], cur[:i + 1])
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if k % max(1, n_frames // 10) == 0:
            print(f'frame {k}/{n_frames}  {hhmm(float(r["hours"]))}', flush=True)
    writer.release()
    print(f'saved {a.out} ({n_frames} frames, {a.seconds:.0f} s)')


if __name__ == '__main__':
    main()
