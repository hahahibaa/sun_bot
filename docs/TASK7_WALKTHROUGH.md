# sun_bot — task 7 walkthrough: sun_brain + plant_monitor

What happens after task 6 (the fake sun through the windows): the robot **learns where the sun
lands at each hour**, then on the next day **drives to it, creeps into it and follows it**, waits
out clouds, and the plant complains when it gets thirsty.

## 1. The pieces

| Node | What it does |
|---|---|
| `sun_sensor` (task 6, improved) | Day clock, sun position (NOAA), sun patches through the 3 windows, now **with pallet shadows**; the robot's panel current and 4 edge light sensors; clouds |
| `sun_brain` (new) | The state machine: LEARN, CHOOSE, GO, SETTLE, TRACK, LOST, NIGHT (+ WAIT) |
| `plant_monitor` (new) | Soil moisture 60 % → dries over the day (faster in the sun); honks < 30 %, quiet > 45 %; `/plant/water` refills |
| `sun_logger`, `sun_report` | CSV logging and the plots/tables below |
| `sun_day.launch.py` (new) | One command: sim + Nav2 + safety stop + sun + plant + brain (+ log) |

The brain only uses what a real robot would have: its AMCL pose, the panel current, the 4 edge
sensors, the lidar, the clock and the sun angles. Never the simulator's ground truth or cloud flag.
All motion goes through Nav2 or `/cmd_vel`, i.e. through the 30 cm safety stop.

## 2. States

```
LEARN (day 1)  every 2 day-hours 07–17: clock FROZEN, visit each sunlit window, sweep for the
               patch, climb the light gradient, save best current + pose  -> learned_sun.yaml
CHOOSE         best learned spot for "now + travel time" (interpolated between learned hours)
GO             Nav2 to it (switches to SETTLE early if a sensor already sees sun nearby)
SETTLE         creep (0.08 m/s) towards the brighter edge sensor until the panel gives 80 % of
               full sun; if no sun at the spot: look around (turn + short moves), then sweep the
               patch's learned path
TRACK          every 3 day-min: nudge towards the brighter edge, re-settle if the panel drops,
               back to CHOOSE when every sensor is at indoor level
LOST           cloud = every reading drops together within 1 s by the same ratio. Hold still up
               to 120 s; leave early only if the dimmed patch slides off (sensors go dark)
WAIT           nothing reachable lit: look around once, re-choose every 10 day-min
NIGHT          after sunset: park at the first morning spot
```

## 3. One full day (visual run, Gazebo + RViz, clouds on)

In RViz: yellow = sun on the floor (what the sensors use), red box = 30 cm safety zone, green
arrow = the brain's target, white text above the robot = state, time, panel current, plant %.

> Gazebo's own light patch is **not** the simulated sun: it is the fixed lamp from task 1 (Fortress
> cannot move lights at runtime, tested in task 6). RViz shows the real sun.

| Time | What happens | RViz | Gazebo |
|---|---|---|---|
| 07:19 | GO: driving to the learned east spot (expect 0.11 A) | ![](run/0719_go_rviz.png) | ![](run/0719_go_gazebo.png) |
| 08:30 | SETTLE/TRACK in the long morning east patch | ![](run/0830_settle_rviz.png) | ![](run/0830_settle_gazebo.png) |
| 10:00 | TRACK: 0.40 A, following the patch as it slides to the east wall | ![](run/1000_track_rviz.png) | ![](run/1000_track_gazebo.png) |
| 11:40 | east patch is now a strip against the wall → moved to the south patch; a cloud dims it | ![](run/1140_track_rviz.png) | ![](run/1140_track_gazebo.png) |
| 13:00 | TRACK in the south patch, all four edges in the sun, 0.37 A | ![](run/1300_track_rviz.png) | ![](run/1300_track_gazebo.png) |
| 13:12 | plant at 30 % → **THIRSTY!** (honk), watered 20 s later | ![](run/1312_thirsty_rviz.png) | ![](run/1312_thirsty_gazebo.png) |
| 14:49 | south patch against the wall → GO to the west spot | ![](run/1449_go_rviz.png) | ![](run/1449_go_gazebo.png) |
| 15:19 | cloud → **LOST**, holding still | ![](run/1519_cloud_lost_rviz.png) | ![](run/1519_cloud_lost_gazebo.png) |
| 16:10 | still LOST (cloud lasts 110 sim s); re-settles at 16:20 when it ends | ![](run/1610_lost_rviz.png) | ![](run/1610_lost_gazebo.png) |
| 17:30 | low evening sun, patch streaks away across the room → WAIT, then night | ![](run/1730_wait_rviz.png) | ![](run/1730_wait_gazebo.png) |

![day plot](day_visual.png)

**52 % of daylight with the panel in the sun, 1.31 A·h sun energy, 36 m driven.**

## 4. Test results

**Test 1, learning** (`config/learned_sun.yaml`, 776 sim s ≈ 13 min wall, clock frozen during
each survey):

| hour | elev | full sun | learned spots (best panel current) |
|---|---|---|---|
| 07:00 | 10° | 0.11 A | south 0.045 A (-4.32, -4.57), east 0.099 A (+5.36, +0.64) |
| 09:00 | 35° | 0.33 A | east 0.325 A (+7.88, +1.17), south 0.327 A (-1.38, -4.76) |
| 11:00 | 53° | 0.45 A | south 0.448 A (-1.17, -4.83); east: none (strip against the wall) |
| 13:00 | 52° | 0.44 A | south 0.444 A (+1.26, -4.78); west: none (strip against the wall) |
| 15:00 | 33° | 0.31 A | west 0.308 A (-7.69, +1.49), south 0.309 A (+1.45, -4.78) |
| 17:00 | 8° | 0.09 A | south 0.085 A (+8.84, -3.39); west: none (patch starts ~7 m in, beyond the 5 m search) |

![learned spots](learned_spots.png)

**Test 2, day-2 run with clouds:** `day2_run.png` (51 % in sun, 1.23 A·h) and the visual run above.

**Test 3, three strategies, same day, same clouds (seed 7):**

| strategy | daylight in sun | sun energy | driven |
|---|---|---|---|
| a) parked at the south spot | 32 % | 0.82 A·h | 5 m |
| b) greedy: wander, follow edge sensors, no map | 0–4 % (two runs) | 0.01–0.02 A·h | 120 m |
| c) sun_brain | 51 % (visual run: 52 %) | 1.23 A·h (1.31) | 50 m (36) |

Greedy wandering almost never crosses a patch (14 sensor samples in sun all day): knowing
*where* the sun lands is what makes the difference. (a–c were run before the pallet shadows were
added; the visual run is with them.)

**Test 4, cloud during TRACK** (`cloud_thirst.png`): tracking at 0.46 A, `/sun/cloud` → LOST at
once, robot moved **0 cm** while the cloud lasted, left only when the dimmed patch had slid off.

**Test 5, thirst** (drying 15× faster): honk on below 30 %; watered to 40 % → **still honking**
(hysteresis); watered to 50 % → honk off.

## 5. Problems found and fixed along the way

- 1 kHz `/clock` overloaded the Python nodes → physics step 4 ms (real-time factor 0.5 → ~1.0).
- Nav2 accepted goals before it was active, and the collision monitor sometimes hung at start-up
  → the brain waits for both to be ACTIVE; the monitor starts 5 s later.
- Creeping into a patch against a wall put the robot inside its own 30 cm stop zone; turning
  away then froze it. Creeping now keeps every obstacle ≥ 0.82 m from the axle (the stop zone's
  corner + margin), so it can always turn and leave.
- LOST waited the full 120 s even after the patch had moved → leaves as soon as the dim patch is
  gone, re-settles when the cloud ends.
- Robot waited 4–13 cm from a patch without seeing it → "look around" before giving up.
- Sun went through the pallets → pallets now cast shadows (`pallet_shadows.png`).
- Plant beep is opt-in (`beep:=true`).

## 6. Known limitations

- Low evening sun throws long streaks that race across the room; the robot often can't follow.
- Patches that hug a wall can't be reached: the safety stop keeps the robot away (by design).
- Gazebo's light patch is the fixed task-1 lamp, not the simulated sun (Fortress limitation).

## 7. Commands

```bash
cd ~/sun_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch sun_bot sun_day.launch.py mode:=learn gui:=false rviz:=false   # day 1: learn (~13 min)
ros2 launch sun_bot sun_day.launch.py clouds:=true                          # day 2: chase the sun
ros2 launch sun_bot sun_day.launch.py strategy:=parked                     # baseline (or greedy)
ros2 service call /sun/cloud std_srvs/srv/Trigger                           # cloud now
ros2 service call /plant/water std_srvs/srv/Trigger                         # water the plant
ros2 launch sun_bot sun_day.launch.py dry_scale:=15.0 beep:=true           # fast thirst, with sound
ros2 launch sun_bot sun_day.launch.py log:=/tmp/day.csv && \
  ros2 run sun_bot sun_report --day-run /tmp/day.csv                        # log + plot a day
```
