# Project Progress Log — Autonomous Restaurant Service Robot

---

## 2026-06-21 — SLAM map came out completely white (no occupied cells)

### Symptom
After driving the full cafe with SLAM Toolbox running, the saved `cafe_map.pgm` was entirely white — no walls, no obstacles marked as occupied anywhere. The map was the correct size (238×238 cells, 11.9m×11.9m, origin [-5.94, -11]) confirming SLAM successfully tracked the robot's position through the drive, but every cell was free space.

### Root causes (two, both fixed)

**Cause 1 — `minimum_laser_range` not set in slam_params.yaml**

`slam_params.yaml` did not set `minimum_laser_range`, so it defaulted to 0.0m in slam_toolbox. Gazebo's ray sensor publishes 0.0m for any reading that falls below the physical sensor minimum (0.12m in our URDF). With `minimum_laser_range: 0.0`, slam_toolbox accepted those 0.0m readings as valid hits — treating them as obstacles at the sensor origin rather than filtering them. This corrupted the scan-matching process and meant no real obstacles were written into the occupancy grid.

The slam_toolbox log showed this warning which was the tell:
```
[WARN] minimum laser range setting (0.0 m) exceeds the capabilities of the used Lidar (0.1 m)
```

**Fix:** Added `minimum_laser_range: 0.12` to `config/slam_params.yaml` to match the URDF sensor `<min>0.12</min>` value.

**Cause 2 — Table pedestals shorter than the lidar scan height**

The lidar is mounted at z=1.307m on the robot (world height ≈1.36m when resting). The table pedestals in `cafe.world` had a height of 0.74m — entirely below the lidar scan plane. The scan plane also passes above the table tops (z=0.74m), the chairs (z=0.9m), and the service counter (z=1.0m). The only objects tall enough to intersect the lidar were the walls (z=0 to z=2.5m).

**Fix:** Extended all 6 table pedestals from 0.74m to 1.5m tall (`center z: 0.37 → 0.75`, `length: 0.74 → 1.5`) in `worlds/cafe.world`. The lidar scan plane at z≈1.36m now passes through each 6cm-radius pedestal column, making all six tables visible as obstacles in the map.

### Diagnostic that confirmed the fix
After rebuilding, with Gazebo and SLAM running but before driving:
```bash
ros2 topic echo /scan --field ranges --once 2>/dev/null | tr ',' '\n' | awk '{if($1+0 < 11.9) count++; total++} END {print "Non-max readings: " count "/" total}'
```
Result: `Non-max readings: 338/360` — 338 of 360 rays returning sub-max-range hits, confirming walls and pedestals are being detected. (The remaining 22 rays point toward open doorway / spawn corridor where nothing blocks them at that angle.)

### Files changed
- `config/slam_params.yaml` — added `minimum_laser_range: 0.12`
- `worlds/cafe.world` — extended all 6 table pedestals to 1.5m height

---

## Step completion status

| Step | Status |
|---|---|
| 1. Sensor plugins (lidar, IMU, wheel encoders) on URDF | Done |
| 2. Load cafe.world and spawn robot | Done |
| 3. SLAM Toolbox — build and save map | In progress (redriving after white-map fix) |
| 4. AMCL localization using saved map | Pending |
| 5. Nav2 stack (costmaps + A* + DWA) | Pending |
