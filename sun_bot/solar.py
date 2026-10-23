"""Solar position from the NOAA solar calculator equations (Meeus-based), no external libraries.

Angles in degrees. Azimuth is measured clockwise from north (90 = east, 180 = south).
Elevation includes NOAA's atmospheric refraction correction. Accuracy is about 0.01 deg
for dates around today, far better than anything the simulation needs.
"""
from datetime import date, datetime, timedelta, timezone
import math


def julian_day(dt_utc):
    """Julian day for a timezone-aware or naive-UTC datetime."""
    if dt_utc.tzinfo is not None:
        dt_utc = dt_utc.astimezone(timezone.utc).replace(tzinfo=None)
    return (dt_utc - datetime(2000, 1, 1, 12)).total_seconds() / 86400.0 + 2451545.0


def _sun_terms(jd):
    """Declination (deg) and equation of time (min) for a Julian day."""
    t = (jd - 2451545.0) / 36525.0
    l0 = (280.46646 + t * (36000.76983 + t * 0.0003032)) % 360.0
    m = 357.52911 + t * (35999.05029 - 0.0001537 * t)
    e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t)
    mr = math.radians(m)
    c = (math.sin(mr) * (1.914602 - t * (0.004817 + 0.000014 * t))
         + math.sin(2 * mr) * (0.019993 - 0.000101 * t)
         + math.sin(3 * mr) * 0.000289)
    omega = math.radians(125.04 - 1934.136 * t)
    app_long = math.radians(l0 + c - 0.00569 - 0.00478 * math.sin(omega))
    eps0 = 23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60
    eps = math.radians(eps0 + 0.00256 * math.cos(omega))
    decl = math.degrees(math.asin(math.sin(eps) * math.sin(app_long)))
    y = math.tan(eps / 2) ** 2
    l0r = math.radians(l0)
    eot = 4 * math.degrees(
        y * math.sin(2 * l0r) - 2 * e * math.sin(mr)
        + 4 * e * y * math.sin(mr) * math.cos(2 * l0r)
        - 0.5 * y * y * math.sin(4 * l0r) - 1.25 * e * e * math.sin(2 * mr))
    return decl, eot


def _refraction(elev):
    """NOAA approximation of atmospheric refraction (deg) for a geometric elevation (deg)."""
    if elev > 85.0:
        return 0.0
    te = math.tan(math.radians(elev))
    if elev > 5.0:
        r = 58.1 / te - 0.07 / te ** 3 + 0.000086 / te ** 5
    elif elev > -0.575:
        r = 1735.0 + elev * (-518.2 + elev * (103.4 + elev * (-12.79 + elev * 0.711)))
    else:
        r = -20.772 / te
    return r / 3600.0


def sun_position(dt, lat, lon):
    """(azimuth, elevation) in degrees for a timezone-aware datetime at lat/lon (deg, E +)."""
    dt_utc = dt.astimezone(timezone.utc)
    decl, eot = _sun_terms(julian_day(dt_utc))
    minutes = dt_utc.hour * 60 + dt_utc.minute + dt_utc.second / 60 + dt_utc.microsecond / 6e7
    tst = (minutes + eot + 4 * lon) % 1440.0               # true solar time, minutes
    ha = tst / 4 - 180.0                                   # hour angle, deg
    latr, declr, har = map(math.radians, (lat, decl, ha))
    cos_zen = math.sin(latr) * math.sin(declr) + math.cos(latr) * math.cos(declr) * math.cos(har)
    zen = math.degrees(math.acos(max(-1.0, min(1.0, cos_zen))))
    elev = 90.0 - zen
    sz = math.sin(math.radians(zen))
    if abs(math.cos(latr) * sz) < 1e-9:
        az = 180.0
    else:
        a = ((math.sin(latr) * math.cos(math.radians(zen)) - math.sin(declr))
             / (math.cos(latr) * sz))
        a = math.degrees(math.acos(max(-1.0, min(1.0, a))))
        az = (a + 180.0) % 360.0 if ha > 0 else (540.0 - a) % 360.0
    return az, elev + _refraction(elev)


def _find_crossing(f, t0, t1, rising, steps=60):
    """First time in [t0, t1] where f crosses 0 upwards (rising) or downwards, by bisection."""
    prev_t, prev_v = t0, f(t0)
    for i in range(1, steps + 1):
        t = t0 + (t1 - t0) * i / steps
        v = f(t)
        if (prev_v < 0 <= v) if rising else (prev_v >= 0 > v):
            lo, hi = prev_t, t
            for _ in range(40):
                mid = lo + (hi - lo) / 2
                if (f(mid) < 0) == rising:
                    lo = mid
                else:
                    hi = mid
            return lo + (hi - lo) / 2
        prev_t, prev_v = t, v
    return None


def sun_events(day, lat, lon, tz):
    """Sunrise, solar noon and sunset (aware datetimes in tz) for a date.

    Sunrise/sunset = upper limb on the horizon (centre at -0.833 deg, NOAA convention).
    """
    start = datetime(day.year, day.month, day.day, tzinfo=tz)

    def elev_minus(h):
        return sun_position(start + timedelta(hours=h), lat, lon)[1] + 0.833

    rise = _find_crossing(elev_minus, 0.0, 12.0, rising=True)
    sset = _find_crossing(elev_minus, 12.0, 24.0, rising=False)
    _, eot = _sun_terms(julian_day(start + timedelta(hours=12)))
    noon_utc_min = 720 - 4 * lon - eot
    noon = (datetime(day.year, day.month, day.day, tzinfo=timezone.utc)
            + timedelta(minutes=noon_utc_min)).astimezone(tz)
    to_dt = (lambda h: None if h is None else start + timedelta(hours=h))
    return to_dt(rise), noon, to_dt(sset)


def tz_from_hours(hours):
    return timezone(timedelta(hours=hours))


def today_in(tz):
    return datetime.now(tz).date()


__all__ = ['sun_position', 'sun_events', 'tz_from_hours', 'today_in', 'julian_day', 'date']
