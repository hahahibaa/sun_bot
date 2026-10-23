"""Sun patches: where direct sunlight through each window lands on a horizontal plane.

Coordinates: map/world frame, +x east, +y north, z up (m). Window data comes from
config/windows.yaml. Pure Python, shared by the sun_sensor node and the sun_report tool.

A window is a rectangular opening in a wall of thickness T. Light gets through only if it
passes both the outer and the inner edge of the opening, so the patch is the intersection of
the two projected openings (this is the shadow cast by the window reveal), clipped to the room.
Boxes in `shadow_boxes` (the pallets) block the light: each casts a convex shadow (its footprint
swept away from the sun by height / tan(elevation)) that is cut out of the patch, so a lit area
is a list of convex pieces. Light that would land on the walls is not drawn (floor only), and
the robot does not shade itself.
"""
import math

import yaml


def load_windows(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    room = cfg.get('room', {'x_min': -9.9, 'x_max': 9.9, 'y_min': -5.9, 'y_max': 5.9})
    return cfg, room


def sun_vector(azimuth_deg, elevation_deg):
    """Unit vector from the ground towards the sun (x east, y north, z up)."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    return (math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))


def window_lit(window, sun):
    """True if the sun is above the horizon and on the outside of this window's wall."""
    f = window['facing_yaw']
    return sun[2] > 0.0 and sun[0] * math.cos(f) + sun[1] * math.sin(f) > 1e-6


def _opening(window, cfg, offset):
    """3D corners of the opening on the plane `offset` m outward from the wall centre line."""
    cx, cy = window['centre']
    f = window['facing_yaw']
    nx, ny = math.cos(f), math.sin(f)          # outward normal
    tx, ty = -ny, nx                            # along the wall
    half = cfg.get('window_width', 2.0) / 2
    lo, hi = cfg.get('sill_height', 1.0), cfg.get('top_height', 2.2)
    px, py = cx + nx * offset, cy + ny * offset
    return [(px - tx * half, py - ty * half, lo), (px + tx * half, py + ty * half, lo),
            (px + tx * half, py + ty * half, hi), (px - tx * half, py - ty * half, hi)]


def project(points3d, sun, plane_z=0.0):
    """Project 3D points along the sun's rays (away from the sun) onto z = plane_z."""
    sx, sy, sz = sun
    return [(x - (z - plane_z) / sz * sx, y - (z - plane_z) / sz * sy) for x, y, z in points3d]


def area(poly):
    """Signed area (CCW positive)."""
    return 0.5 * sum(poly[i - 1][0] * poly[i][1] - poly[i][0] * poly[i - 1][1]
                     for i in range(len(poly)))


def ccw(poly):
    return poly if area(poly) >= 0 else poly[::-1]


def clip(subject, clipper):
    """Sutherland-Hodgman: part of polygon `subject` inside convex polygon `clipper`."""
    clipper = ccw(clipper)
    out = list(subject)
    for i in range(len(clipper)):
        if not out:
            break
        ax, ay = clipper[i - 1]
        bx, by = clipper[i]

        def inside(p):
            return (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax) >= -1e-12

        def cross(p, q):
            dx1, dy1 = q[0] - p[0], q[1] - p[1]
            dx2, dy2 = bx - ax, by - ay
            den = dx1 * dy2 - dy1 * dx2
            t = ((ax - p[0]) * dy2 - (ay - p[1]) * dx2) / den
            return (p[0] + t * dx1, p[1] + t * dy1)

        inp, out = out, []
        for j in range(len(inp)):
            p, q = inp[j - 1], inp[j]
            if inside(q):
                if not inside(p):
                    out.append(cross(p, q))
                out.append(q)
            elif inside(p):
                out.append(cross(p, q))
    return out


def clip_halfplane(poly, a, b, keep_left=True):
    """Part of a convex polygon on the left (or right) of the directed line a -> b."""
    def side(p):
        v = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        return v if keep_left else -v

    out = []
    for j in range(len(poly)):
        p, q = poly[j - 1], poly[j]
        sp, sq = side(p), side(q)
        if sq >= 0:
            if sp < 0:
                k = sp / (sp - sq)
                out.append((p[0] + k * (q[0] - p[0]), p[1] + k * (q[1] - p[1])))
            out.append(q)
        elif sp >= 0:
            k = sp / (sp - sq)
            out.append((p[0] + k * (q[0] - p[0]), p[1] + k * (q[1] - p[1])))
    return out


def subtract(poly, hole):
    """Convex polygon minus a convex hole, as a list of convex pieces."""
    hole = ccw(hole)
    overlap = clip(poly, hole)
    if len(overlap) < 3 or abs(area(overlap)) < 1e-6:
        return [poly]                          # the shadow misses this piece: keep it whole
    pieces, rest = [], ccw(poly)
    for i in range(len(hole)):
        a, b = hole[i - 1], hole[i]
        outside = clip_halfplane(rest, a, b, keep_left=False)
        if len(outside) >= 3 and abs(area(outside)) > 1e-6:
            pieces.append(ccw(outside))
        rest = clip_halfplane(rest, a, b, keep_left=True)
        if len(rest) < 3:
            break
    return pieces


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def box_shadow(box, sun, plane_z=0.0):
    """Shadow of an upright box on z = plane_z: points whose ray to the sun hits the box."""
    h = box.get('height', 1.0)
    if sun[2] <= 0 or plane_z >= h:
        return []
    cx, cy = box['centre']
    sx, sy = box['size']
    foot = [(cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy - sy / 2),
            (cx + sx / 2, cy + sy / 2), (cx - sx / 2, cy + sy / 2)]
    t = (h - plane_z) / sun[2]
    ox, oy = -sun[0] * t, -sun[1] * t
    return convex_hull(foot + [(x + ox, y + oy) for x, y in foot])


def room_poly(room):
    return [(room['x_min'], room['y_min']), (room['x_max'], room['y_min']),
            (room['x_max'], room['y_max']), (room['x_min'], room['y_max'])]


def window_patch(window, cfg, room, sun, plane_z=0.0):
    """Sun patch (CCW list of (x, y)) of one window on z = plane_z, or [] if not lit."""
    if not window_lit(window, sun):
        return []
    t = cfg.get('wall_thickness', 0.2)
    outer = ccw(project(_opening(window, cfg, +t / 2), sun, plane_z))
    inner = ccw(project(_opening(window, cfg, -t / 2), sun, plane_z))
    patch = clip(clip(outer, inner), room_poly(room))
    return ccw(patch) if len(patch) >= 3 and abs(area(patch)) > 1e-6 else []


def lit_pieces(window, cfg, room, sun, plane_z=0.0):
    """Sunlit area of one window on z = plane_z: the patch minus the pallets' shadows."""
    pieces = [window_patch(window, cfg, room, sun, plane_z)]
    if not pieces[0]:
        return []
    for box in cfg.get('shadow_boxes', []):
        sh = box_shadow(box, sun, plane_z)
        if sh:
            pieces = [q for p in pieces for q in subtract(p, sh)]
    return pieces


def all_patches(cfg, room, sun, plane_z=0.0):
    """{window name: [convex pieces]} of sunlit floor for the lit windows."""
    out = {}
    for w in cfg['windows']:
        pieces = lit_pieces(w, cfg, room, sun, plane_z)
        if pieces:
            out[w['name']] = pieces
    return out


def pieces_area(pieces):
    return sum(area(p) for p in pieces)


def pieces_centroid(pieces):
    a = pieces_area(pieces)
    cs = [(centroid(p), area(p)) for p in pieces]
    return (sum(c[0] * w for c, w in cs) / a, sum(c[1] * w for c, w in cs) / a)


def inside_any(point, pieces):
    return any(inside(point, p) for p in pieces)


def distance_to_pieces(point, pieces):
    return min(distance_to(point, p) for p in pieces)


def inside(point, poly):
    """Point in convex CCW polygon."""
    x, y = point
    for i in range(len(poly)):
        ax, ay = poly[i - 1]
        bx, by = poly[i]
        if (bx - ax) * (y - ay) - (by - ay) * (x - ax) < 0:
            return False
    return bool(poly)


def centroid(poly):
    a = area(poly)
    if abs(a) < 1e-12:
        return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
    cx = cy = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i - 1]
        x1, y1 = poly[i]
        c = x0 * y1 - x1 * y0
        cx += (x0 + x1) * c
        cy += (y0 + y1) * c
    return (cx / (6 * a), cy / (6 * a))


def distance_to(point, poly):
    """0 if inside, else the distance from point to the polygon's boundary."""
    if inside(point, poly):
        return 0.0
    best = math.inf
    px, py = point
    for i in range(len(poly)):
        ax, ay = poly[i - 1]
        bx, by = poly[i]
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
        best = min(best, math.hypot(px - ax - t * dx, py - ay - t * dy))
    return best


def body_to_world(pose, bx, by):
    """Point (bx, by) in the robot frame -> world, pose = (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * bx - s * by, y + s * bx + c * by)
