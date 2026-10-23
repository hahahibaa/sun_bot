# Results: three strategies, same day

Final code (pallet shadows, look-around, cloud handling), headless, Jabalpur 23 Oct, 07:00–17:45,
1 day-minute per sim-second, the same random clouds (seed 7: 102 sim-s at 10:38, 110 sim-s at
15:20) for every run, robot starting at the room centre.

| Strategy | Daylight with the panel in the sun | Sun energy | Total energy | Distance driven |
|---|---|---|---|---|
| a) Park at the south window spot | 32.3 % | 0.82 A·h | 0.92 A·h | 4.9 m |
| b) Greedy: wander, follow the edge sensors, no map | 18.9 % | 0.42 A·h | 0.51 A·h | 97.4 m |
| c) **sun_brain** (learned map of sun spots) | **49.5 %** | **1.27 A·h** | **1.37 A·h** | 37.0 m |

- "In the sun" = at least half of the panel inside a sun patch (simulator ground truth).
- Sun energy = panel current above the indoor-light floor (0.01 A), integrated over the day;
  total energy includes that floor.
- sun_brain collects **54 % more sun energy than parking** and **3x more than greedy**, while
  driving less than half as far as greedy.

Run files: `run_parked.csv`, `run_greedy.csv`, `run_brain.csv`; plots `strategies.png`
(all three) and `day2_run.png` (sun_brain with its state timeline).

## Why greedy fails

Indoor light is the same almost everywhere (about 300 lux), so away from a sun patch the light
sensors give no hint of which way the sun is: there is no gradient to follow until the robot is
already touching a patch. And the patches are small: at any moment only about 1 % of the floor
is sunlit (median 1.2 %, between 0.7 and 2.4 % for half of the day), so a robot wandering blindly
almost never crosses one. In this run greedy drove 97 m and found no sun at all until 14:06,
when it happened to bump into the large afternoon patch from the west window; in two earlier
runs of the same strategy it got 0 % and 4 %. Its result is mostly luck. sun_brain knows from
day 1 where the patches will be at each hour, so it drives straight there and only uses the edge
sensors for the last half-metre.

## Why parking is not enough

Each window's patch only covers its parking spot for part of the day (east 08:45–09:40, south
10:05–14:05, west 14:12–15:07), because the patches slide across the room as the sun moves. The
south spot is the best fixed choice, and it still misses the whole morning and late afternoon.

## Earlier runs (for reference)

Before the final fixes (no pallet shadows, older cloud and look-around logic) the same comparison
gave: parked 32.2 %, greedy 0–4 %, sun_brain 51.3 % (1.23 A·h). The visual run with the final
code gave 52 % (1.31 A·h). sun_brain is consistently around 50 %, parked around 32 %; greedy
varies between runs from 0 to 19 %.
