# sun_bot: a robot that keeps a potted plant in the sun

A small differential-drive robot carries a plant around a closed warehouse (20 x 12 m, roof,
three 2 m windows facing east, south and west, four pallets). The sun only gets in through the
windows, so the bright patches on the floor move, shrink and vanish during the day. The robot's
job is to keep the plant (and its 10 W solar panel, which we use as a light meter) in a patch.
Everything runs in simulation: ROS 2 Humble, Gazebo Fortress, Nav2, on WSL2.

## What each prototype proved

| # | Prototype | What it proved | Key numbers |
|---|---|---|---|
| 1 | Warehouse world | A reproducible room generated from one script | 20 x 12 x 3 m, 3 windows (sill 1.0, top 2.2 m), 4 pallets |
| 2 | Robot + ROS 2 bridge | Our own robot drives and senses in Gazebo, all topics in ROS 2 | 360-ray 10 m lidar at 10 Hz; teleop works |
| 3 | Mapping (SLAM) | A clean map can be built automatically | walls within 1-7 cm, pallets within 4 cm of truth |
| 4 | Windows on the map | Windows and parking spots tagged from the building plan | 3 parking spots, 0.8-0.9 m from the walls |
| 5 | Navigation (Nav2) | The robot drives itself to every window and stops safely | 3/3 reached, ~0.1 m and < 5 deg error; AMCL within 4 cm; safety stop halts 0.13 m from a box that appears in front |
| 6 | Fake sun | A physically sensible sun and light sensors to chase | NOAA sun: sunrise 06:09, noon 55 deg, sunset 17:39 (Jabalpur, 23 Oct); patches follow the windows through the day, pallets cast shadows |
| 7 | sun_brain | Learning *where* the sun lands beats both standing still and wandering | see the table below |
| 8 | Demo | A repeatable scripted day, recorded, plus replays from the logs | `demo:=true`; videos linked from the README |

## The main result

| Strategy (same day, same clouds) | Panel in the sun | Sun energy | Driven |
|---|---|---|---|
| Park at the best window spot (south) | 32 % of daylight | 0.82 A·h | 5 m |
| Greedy: wander and follow the light sensors | 19 % (0-19 % over 3 runs) | 0.42 A·h | 97 m |
| **sun_brain: learn the sun spots, then chase** | **50 %** (49.5-52 % over 3 runs) | **1.27 A·h** | 37 m |

**sun_brain gets 54 % more sun energy than the best parking spot and 3x more than greedy.**
Full table and method: [results.md](results.md).

Same simulated day, same clouds, all headless. "In sun" = at least half the panel inside a sun
patch (ground truth). Energy = panel current above the indoor floor, integrated over the day.

Why it works: the parking spots next to the windows are only lit for an hour or two (east
08:45-09:40, south 10:05-14:05, west 14:12-15:07), because the patches slide across the room as
the sun moves. On day 1 the robot surveys every sunlit window every two hours and remembers the
best spot; on day 2 it interpolates between those spots, drives there with Nav2, creeps into the
light using four edge light sensors, and follows the patch as it moves.

Why the greedy baseline fails: indoor light is the same everywhere (about 300 lux), so away from
a patch the light sensors carry no hint of where the sun is: there is no gradient to climb until
the robot is already touching a patch. A patch covers only 1-2 % of the floor most of the day,
so wandering blindly almost never finds one, and when it does, it usually drives straight past.

## Known limitations

- Low evening sun throws long streaks that race across the room; the robot often can't keep up.
- Patches pressed against a wall can't be reached: the 30 cm safety stop (by design) keeps the
  robot's body away from walls, and the panel sits in the middle of the body.
- Day 1 learning freezes the simulated clock during each survey; a real robot would need a few
  sunny days, or learn one hour at a time.
- The sun and the light sensors are simulated (geometric model, no reflections, no diffuse
  daylight changes, no robot self-shadow); Gazebo itself cannot show the moving sun (Fortress
  cannot move lights at runtime), so RViz is the sun view.
- Clouds are detected from the sensor pattern (everything dims together); a very slow, thin
  cloud would look like a moving patch.

## What I'd do next with real hardware

1. **Measure before building**: log a real light meter at a few points by each window for a week;
   check how sharp the patch edges are and how much diffuse light there is.
2. **Swap the fake sensors for real ones**: a small solar cell with a current sensor (e.g. an
   INA219) as the panel, and 4 cheap light sensors (e.g. BH1750) on the body edges.
3. **Keep the learned table, update it every day**: store the best spot per hour and nudge it
   every time the robot finds sun, so it adapts to seasons without a separate learning day.
4. **Use the sun position, not just the table**: with the room geometry known, predict where the
   patch should be from the sun angle, and only use the sensors for the last metre.
5. **Safety and power**: bumpers in addition to the lidar stop zone, a docking/charging spot,
   and a watering reminder (or pump) driven by a real soil-moisture probe.
