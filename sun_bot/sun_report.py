"""Offline sun checks for the warehouse (no simulation needed), plus the panel-current plot.

    ros2 run sun_bot sun_report                       # table + docs/sun_patches.png + distances
    ros2 run sun_bot sun_report --date 2026-12-21     # another day
    ros2 run sun_bot sun_report --panel panel_east.csv panel_south.csv panel_west.csv
                                                      # docs/panel_current.png from sun_logger CSVs
    ros2 run sun_bot sun_report --learned             # docs/learned_spots.png (learned_sun.yaml)
    ros2 run sun_bot sun_report --day-run day2.csv    # docs/day2_run.png: current + states
    ros2 run sun_bot sun_report --compare parked.csv greedy.csv brain.csv
                                                      # strategy table + docs/strategies.png
    ros2 run sun_bot sun_report --compare-seeds ~/sun_ws/docs/seeds
                                                      # mean / min-max over seeds + strategies.png
"""
import argparse
import csv
from datetime import datetime, timedelta
import math
import os

from ament_index_python.packages import get_package_share_directory
import yaml

from sun_bot.solar import sun_events, sun_position, today_in, tz_from_hours
from sun_bot.sun_geometry import (all_patches, body_to_world, distance_to_pieces, inside,
                                  load_windows, pieces_area, pieces_centroid, sun_vector)

LAT, LON, UTC_OFFSET = 23.18, 79.95, 5.5
PANEL_HEIGHT, PANEL_X, PANEL_SIZE = 0.18, -0.08, 0.25


def at(day, tz, hours):
    return datetime(day.year, day.month, day.day, tzinfo=tz) + timedelta(hours=hours)


def panel_fraction(pose, polys, n=7):
    hits = 0
    for i in range(n):
        for j in range(n):
            pt = body_to_world(pose, PANEL_X + PANEL_SIZE * ((i + 0.5) / n - 0.5),
                               PANEL_SIZE * ((j + 0.5) / n - 0.5))
            hits += any(inside(pt, p) for p in polys)
    return hits / (n * n)


def report(day, cfg, room, out_dir):
    tz = tz_from_hours(UTC_OFFSET)
    rise, noon, sset = sun_events(day, LAT, LON, tz)
    noon_el = sun_position(noon, LAT, LON)[1]
    print(f'Jabalpur ({LAT} N, {LON} E), {day}: sunrise {rise:%H:%M}, solar noon {noon:%H:%M} '
          f'(elevation {noon_el:.1f} deg), sunset {sset:%H:%M} IST\n')

    print(f'{"time":5s} {"azimuth":>8s} {"elev":>6s}  {"sun through":12s} '
          'patch area on floor (m2)')
    for h in range(7, 18):
        az, el = sun_position(at(day, tz, h), LAT, LON)
        floor = all_patches(cfg, room, sun_vector(az, el), 0.0) if el > 0 else {}
        areas = ', '.join(f'{k} {pieces_area(v):.2f}' for k, v in floor.items())
        print(f'{h:02d}:00 {az:8.1f} {el:6.1f}  {", ".join(floor) or "-":12s} {areas}')

    # distances from each parking spot to that window's patch (at panel height)
    print('\nPatch vs parking spot (patch at panel height 0.18 m; "gap" = parking spot (base_link)'
          ' to nearest patch edge, 0 = inside;\n"centre" = distance to patch centroid; '
          '"panel" = fraction of the panel in the sun when parked)')
    windows = {w['name']: w for w in cfg['windows']}
    for name, w in windows.items():
        p = w['parking']
        pose = (p['x'], p['y'], p['yaw'])
        print(f'\n{name} window, parking spot ({p["x"]:+.1f}, {p["y"]:+.1f}):')
        print(f'  {"time":5s} {"centroid (x, y)":>17s} {"area":>5s} {"centre":>7s} {"gap":>5s} '
              f'{"panel":>5s}')
        for half_hours in range(7 * 2, 17 * 2 + 1):
            h = half_hours / 2
            az, el = sun_position(at(day, tz, h), LAT, LON)
            patch = (all_patches(cfg, room, sun_vector(az, el), PANEL_HEIGHT).get(name)
                     if el > 0 else None)
            if not patch:
                continue
            c = pieces_centroid(patch)
            print(f'  {int(h):02d}:{int(round(h % 1 * 60)):02d} ({c[0]:+6.2f}, {c[1]:+6.2f}) '
                  f'{pieces_area(patch):5.2f} {math.hypot(c[0] - p["x"], c[1] - p["y"]):6.2f}m '
                  f'{distance_to_pieces((p["x"], p["y"]), patch):4.2f}m '
                  f'{panel_fraction(pose, patch):5.0%}')
    plot_patches(day, tz, cfg, room, os.path.join(out_dir, 'sun_patches.png'))


def draw_room(ax, cfg, room):
    """Saved map as background, windows, parking spots."""
    import numpy as np
    from PIL import Image
    share = get_package_share_directory('sun_bot')
    meta = yaml.safe_load(open(os.path.join(share, 'maps', 'warehouse.yaml')))
    img = np.array(Image.open(os.path.join(share, 'maps', meta['image'])))
    ox, oy = meta['origin'][:2]
    res = meta['resolution']
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, alpha=0.5,
              extent=(ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res))
    half = cfg.get('window_width', 2.0) / 2
    for w in cfg['windows']:
        cx, cy = w['centre']
        f = w['facing_yaw']
        tx, ty = -math.sin(f), math.cos(f)
        ax.plot([cx - tx * half, cx + tx * half], [cy - ty * half, cy + ty * half],
                color='deepskyblue', lw=5, solid_capstyle='butt')
        p = w['parking']
        ax.plot(p['x'], p['y'], 'o', color='black', ms=5)
        ax.annotate(w['name'], (p['x'], p['y']), textcoords='offset points',
                    xytext=(0, 8), ha='center', fontsize=9)
    ax.set_xlim(room['x_min'] - 0.6, room['x_max'] + 0.6)
    ax.set_ylim(room['y_min'] - 0.6, room['y_max'] + 0.6)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m, east)')
    ax.set_ylabel('y (m, north)')


def plot_patches(day, tz, cfg, room, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig, ax = plt.subplots(figsize=(11, 7))
    draw_room(ax, cfg, room)
    # morning blue -> noon yellow -> evening red
    hours = {8: '#2c7fb8', 10: '#41b6c4', 12: '#f5c400', 14: '#fd8d3c', 16: '#d7301f'}
    for h, colour in hours.items():
        az, el = sun_position(at(day, tz, h), LAT, LON)
        floor = all_patches(cfg, room, sun_vector(az, el), 0.0)
        for name, pieces in floor.items():
            for poly in pieces:
                ax.add_patch(Polygon(poly, closed=True, facecolor=colour, edgecolor='none',
                                     alpha=0.55))
            c = pieces_centroid(pieces)
            ax.annotate(f'{h:02d}:00', c, ha='center', va='center', fontsize=8, color='black')
        ax.plot([], [], 's', color=colour, ms=10, alpha=0.7,
                label=f'{h:02d}:00  az {az:.0f}, el {el:.0f}  ({", ".join(floor) or "-"})')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=9, frameon=False)
    ax.set_title(f'Sun patches on the floor, Jabalpur {day} (IST)')
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    print(f'\nsaved {path}')


def plot_panel(csv_paths, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                   gridspec_kw={'height_ratios': [3, 1]})
    colours = {'east': 'tab:orange', 'south': 'tab:red', 'west': 'tab:purple'}
    for p in csv_paths:
        with open(p) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        label = rows[0].get('label') or os.path.splitext(os.path.basename(p))[0]
        hrs = [float(r['hours']) for r in rows]
        ax1.plot(hrs, [float(r['panel_current']) for r in rows], lw=1.2,
                 color=colours.get(label), label=f'parked at {label} window')
        ax2.plot(hrs, [float(r['elevation']) for r in rows], color='gray', lw=1)
    ax1.set_ylabel('panel current (A)')
    ax1.grid(alpha=0.3)
    ax1.legend()
    ax1.set_title('Panel current through the day, robot parked at each window\'s parking spot')
    ax2.set_ylabel('sun elev (deg)')
    ax2.set_xlabel('time of day (h, IST)')
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'saved {path}')


HOUR_COLOURS = {7: '#253494', 9: '#2c7fb8', 11: '#41b6c4', 13: '#f5c400', 15: '#fd8d3c',
                17: '#d7301f'}
STATE_COLOURS = {'INIT': '#bbbbbb', 'CHOOSE': '#9e9ac8', 'GO': '#6baed6', 'SETTLE': '#fdae6b',
                 'TRACK': '#31a354', 'LOST': '#636363', 'WAIT': '#d9d9d9', 'NIGHT': '#08306b',
                 'EXPLORE': '#c994c7', 'PARKED': '#74c476', 'LEARN': '#fd8d3c'}


def hour_colour(h):
    return HOUR_COLOURS.get(int(round(h)), '#888888')


def read_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def plot_learned(learned_path, cfg, room, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    with open(learned_path) as f:
        doc = yaml.safe_load(f)
    day = datetime.strptime(doc['learned_on'][:10], '%Y-%m-%d').date()
    tz = tz_from_hours(UTC_OFFSET)
    fig, ax = plt.subplots(figsize=(11, 7.5))
    draw_room(ax, cfg, room)
    marks = {'east': 'o', 'south': 's', 'west': '^'}
    for k, smp in enumerate(doc['samples']):
        h, colour = smp['hour'], hour_colour(smp['hour'])
        az, el = sun_position(at(day, tz, h), LAT, LON)
        for name, pieces in all_patches(cfg, room, sun_vector(az, el), PANEL_HEIGHT).items():
            for poly in pieces:
                ax.add_patch(Polygon(poly, closed=True, facecolor=colour, edgecolor=colour,
                                     alpha=0.13, lw=0.5, ls='--'))
        for name, e in smp['windows'].items():
            if not e.get('found'):
                w = next(w for w in cfg['windows'] if w['name'] == name)
                ax.plot(w['parking']['x'], w['parking']['y'], 'x', color=colour, ms=12, mew=2)
                ax.annotate(f'{h:02.0f}:00 none', (w['parking']['x'], w['parking']['y']),
                            textcoords='offset points', xytext=(8, -14), fontsize=7,
                            color=colour)
                continue
            ax.plot(e['x'], e['y'], marks.get(name, 'o'), color=colour, ms=10,
                    mec='black' if e.get('reachable') else colour,
                    mfc=colour if e.get('reachable') else 'white', mew=1.5)
            ax.arrow(e['x'], e['y'], 0.45 * math.cos(e['yaw']), 0.45 * math.sin(e['yaw']),
                     width=0.03, color=colour, length_includes_head=True)
            ax.annotate(f'{h:02.0f}:00 {e["best_current"]:.2f} A', (e['x'], e['y']),
                        textcoords='offset points', fontsize=8,
                        xytext=(8, 6) if k % 2 == 0 else (8, -15))   # alternate: no overlap
    for h, c in HOUR_COLOURS.items():
        ax.plot([], [], 's', color=c, ms=9, label=f'{h:02d}:00')
    ax.plot([], [], 'o', color='gray', mec='black', label='learned best spot (east)')
    ax.plot([], [], 's', color='gray', mec='black', label='(south)')
    ax.plot([], [], '^', color='gray', mec='black', label='(west)')
    ax.plot([], [], 'x', color='gray', label='searched, no sun reachable')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=5, fontsize=8,
              frameon=False)
    ax.set_title('Learned sun spots (day 1). Dashed: true patch at each hour (for reference)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'saved {path}')


def run_metrics(rows, indoor=0.01):
    """% of daylight time in sun, sun energy (A h above the indoor floor), distance driven."""
    t_day = t_sun = energy = energy_tot = dist = 0.0
    prev = None
    for r in rows:
        if prev is not None:
            dh = float(r['hours']) - float(prev['hours'])
            if 0 < dh < 0.2 and float(r['elevation']) > 0:
                t_day += dh
                if float(r['panel_fraction']) >= 0.5:
                    t_sun += dh
                cur = float(r['panel_current'])
                energy += max(0.0, cur - indoor) * dh
                energy_tot += cur * dh
            step = math.hypot(float(r['x']) - float(prev['x']), float(r['y']) - float(prev['y']))
            if step > 0.002:
                dist += step
        prev = r
    return {'daylight_h': t_day, 'in_sun_pct': 100 * t_sun / t_day if t_day else 0.0,
            'sun_energy_Ah': energy, 'total_energy_Ah': energy_tot, 'distance_m': dist}


def spans(rows, key):
    """[(start hour, end hour, value)] of runs of the same value in a column."""
    out = []
    for r in rows:
        h, v = float(r['hours']), r[key]
        if out and out[-1][2] == v:
            out[-1][1] = h
        else:
            out.append([h, h, v])
    return out


def plot_day_run(csv_path, path, title=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = read_rows(csv_path)
    hrs = [float(r['hours']) for r in rows]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8), sharex=True,
                                        gridspec_kw={'height_ratios': [4, 1, 1.4]})
    for a, b, v in spans(rows, 'cloudy'):
        if v == '1':
            ax1.axvspan(a, b, color='gray', alpha=0.25, lw=0)
    full = [0.55 * max(0.0, math.sin(math.radians(float(r['elevation'])))) + 0.01 for r in rows]
    ax1.plot(hrs, full, color='gray', lw=1, ls='--', label='full-sun current (panel spec)')
    ax1.plot(hrs, [0.8 * f + 0.002 for f in full], color='gray', lw=0.8, ls=':',
             label='80 % settle target')
    ax1.plot(hrs, [float(r['panel_current']) for r in rows], color='tab:red', lw=1,
             label='panel current')
    ax1.fill_between([], [], color='gray', alpha=0.25, label='cloud (ground truth)')
    ax1.set_ylabel('panel current (A)')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(alpha=0.3)
    m = run_metrics(rows)
    ax1.set_title(title or f'Day-2 run: {m["in_sun_pct"]:.0f} % of daylight in sun, '
                           f'{m["sun_energy_Ah"]:.2f} A h sun energy, '
                           f'{m["distance_m"]:.0f} m driven')
    for a, b, v in spans(rows, 'state'):
        ax2.axvspan(a, b, color=STATE_COLOURS.get(v, '#cccccc'), lw=0)
    for st, c in STATE_COLOURS.items():
        if any(r['state'] == st for r in rows):
            ax2.plot([], [], 's', color=c, label=st)
    ax2.set_yticks([])
    ax2.set_ylabel('state')
    ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=9, fontsize=8,
               frameon=False)
    moist = [float(r['moisture']) for r in rows]
    ax3.plot(hrs, moist, color='tab:green', lw=1.2, label='soil moisture')
    ax3.axhline(30, color='tab:red', lw=0.8, ls='--')
    ax3.axhline(45, color='tab:orange', lw=0.8, ls='--')
    for a, b, v in spans(rows, 'thirsty'):
        if v == '1':
            ax3.axvspan(a, b, color='tab:red', alpha=0.15, lw=0)
    ax3.set_ylabel('plant %')
    ax3.set_xlabel('time of day (h, IST)')
    ax3.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'saved {path}')


def compare(csv_paths, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    print(f'{"strategy":10s} {"daylight h":>10s} {"in sun %":>9s} {"sun A h":>8s} '
          f'{"total A h":>9s} {"driven m":>9s}')
    fig, ax = plt.subplots(figsize=(12, 5))
    for p in csv_paths:
        rows = read_rows(p)
        label = rows[0]['label'] or os.path.basename(p)
        m = run_metrics(rows)
        print(f'{label:10s} {m["daylight_h"]:10.2f} {m["in_sun_pct"]:9.1f} '
              f'{m["sun_energy_Ah"]:8.3f} {m["total_energy_Ah"]:9.3f} {m["distance_m"]:9.1f}')
        ax.plot([float(r['hours']) for r in rows], [float(r['panel_current']) for r in rows],
                lw=1, label=f'{label}: {m["in_sun_pct"]:.0f} % in sun, '
                            f'{m["sun_energy_Ah"]:.2f} A h, {m["distance_m"]:.0f} m')
    ax.set_xlabel('time of day (h, IST)')
    ax.set_ylabel('panel current (A)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title('Same day, same clouds: three strategies')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'saved {path}')


def compare_seeds(folder, path):
    """Mean and min-max per strategy over run_<strategy>_s<seed>.csv files; bar chart with
    min-max error bars and one dot per seed."""
    import glob
    import re
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    runs = {}
    for p in sorted(glob.glob(os.path.join(folder, 'run_*_s*.csv'))):
        m = re.match(r'run_(\w+?)_s(\d+)\.csv$', os.path.basename(p))
        if m:
            met = run_metrics(read_rows(p))
            if met['daylight_h'] < 10.0:                      # day not finished (yet)
                print(f'skipped {os.path.basename(p)}: only {met["daylight_h"]:.1f} h logged')
                continue
            runs.setdefault(m.group(1), {})[int(m.group(2))] = met
    order = [k for k in ('parked', 'greedy', 'brain') if k in runs] + \
        sorted(k for k in runs if k not in ('parked', 'greedy', 'brain'))
    names = {'parked': 'parked (south spot)', 'greedy': 'greedy (no map)',
             'brain': 'sun_brain'}
    metrics = [('in_sun_pct', 'daylight in the sun (%)', '{:.1f}'),
               ('sun_energy_Ah', 'sun energy (A h)', '{:.2f}'),
               ('distance_m', 'distance driven (m)', '{:.0f}')]
    summary = {}
    print(f'{"strategy":22s} {"seeds":>5s}  ' + '  '.join(f'{lbl:>28s}' for _, lbl, _ in metrics))
    for k in order:
        seeds = sorted(runs[k])
        row = []
        for key, _, fmt in metrics:
            vals = [runs[k][sd][key] for sd in seeds]
            mean = sum(vals) / len(vals)
            summary.setdefault(k, {})[key] = (mean, min(vals), max(vals), vals)
            row.append(f'{fmt.format(mean)} ({fmt.format(min(vals))}-{fmt.format(max(vals))})')
        print(f'{names.get(k, k):22s} {len(seeds):5d}  ' + '  '.join(f'{c:>28s}' for c in row))
    print('seeds:', ', '.join(f'{k}: {sorted(runs[k])}' for k in order))
    colours = {'parked': '#9e9ac8', 'greedy': '#fd8d3c', 'brain': '#31a354'}
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    for ax, (key, label, _) in zip(axes, metrics):
        for i, k in enumerate(order):
            mean, lo, hi, vals = summary[k][key]
            ax.bar(i, mean, color=colours.get(k, 'gray'), alpha=0.85, width=0.6)
            ax.errorbar(i, mean, yerr=[[mean - lo], [hi - mean]], color='black', capsize=8,
                        lw=1.5)
            ax.plot([i] * len(vals), vals, 'o', color='black', ms=4, alpha=0.6)
        ax.set_xticks(range(len(order)))
        ax.set_xticklabels([names.get(k, k) for k in order], fontsize=9)
        ax.set_title(label)
        ax.grid(axis='y', alpha=0.3)
    n = len(runs[order[0]]) if order else 0
    fig.suptitle(f'Three strategies over {n} cloud seeds: bar = mean, line = min-max, '
                 f'dots = individual days')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'saved {path}')
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--date', default='', help='YYYY-MM-DD (default: today in IST)')
    ap.add_argument('--out-dir', default=os.path.expanduser('~/sun_ws/docs'))
    ap.add_argument('--windows', default=os.path.join(
        get_package_share_directory('sun_bot'), 'config', 'windows.yaml'))
    ap.add_argument('--panel', nargs='+', metavar='CSV',
                    help='plot panel current from sun_logger CSVs instead')
    ap.add_argument('--learned', nargs='?', metavar='YAML',
                    const=os.path.expanduser('~/sun_ws/src/sun_bot/config/learned_sun.yaml'),
                    help='plot learned spots (default: config/learned_sun.yaml)')
    ap.add_argument('--day-run', metavar='CSV', help='plot one sun_brain day')
    ap.add_argument('--compare', nargs='+', metavar='CSV', help='compare strategy runs')
    ap.add_argument('--compare-seeds', metavar='DIR',
                    help='mean / min-max over run_<strategy>_s<seed>.csv files in DIR')
    a = ap.parse_args()
    if a.panel:
        plot_panel(a.panel, os.path.join(a.out_dir, 'panel_current.png'))
        return
    if a.compare_seeds:
        compare_seeds(a.compare_seeds, os.path.join(a.out_dir, 'strategies.png'))
        return
    if a.learned or a.day_run or a.compare:
        cfg, room = load_windows(a.windows)
        if a.learned:
            plot_learned(a.learned, cfg, room, os.path.join(a.out_dir, 'learned_spots.png'))
        if a.day_run:
            plot_day_run(a.day_run, os.path.join(a.out_dir, 'day2_run.png'))
        if a.compare:
            compare(a.compare, os.path.join(a.out_dir, 'strategies.png'))
        return
    day = (datetime.strptime(a.date, '%Y-%m-%d').date() if a.date
           else today_in(tz_from_hours(UTC_OFFSET)))
    cfg, room = load_windows(a.windows)
    report(day, cfg, room, a.out_dir)


if __name__ == '__main__':
    main()
