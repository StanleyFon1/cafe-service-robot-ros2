# Autonomous Restaurant Service Robot — Technical Report

## Central Deliverable

The primary objective of this project is to demonstrate that a differential-drive service robot can navigate from a fixed start point (service counter) to a goal point (customer table) while dynamically avoiding obstacles in its path. The robot must do this autonomously — without human teleoperation — using only onboard lidar sensing and a pre-built map of the cafe environment.

Every algorithm described in this report exists to serve that single capability. SLAM builds the map the robot navigates within. AMCL tells the robot where it is on that map in real time. A* computes the safest route to the table. DWB steers the robot along that route while reacting to objects not in the original map. The sections below explain each layer, why it was chosen over alternatives, and the engineering decisions made to make it work reliably in a resource-constrained virtual machine environment.

---

## 1. Full Navigation Pipeline

### 1.1 SLAM Toolbox — Building the Map

Before autonomous navigation is possible, the robot needs a map. SLAM (Simultaneous Localisation and Mapping) solves the problem of building that map while the robot is being manually driven through the space, without any prior knowledge of the environment.

The implementation uses ROS 2's `slam_toolbox` package in `online_async` mode. On each lidar scan callback, SLAM Toolbox performs two operations in parallel:

**Odometry prediction.** The diff-drive plugin publishes wheel encoder odometry to the `odom → base_link` transform at approximately 30 Hz. SLAM reads this transform to get a rough dead-reckoning estimate of the robot's motion since the previous scan. This is computationally cheap but accumulates error over distance because wheels slip slightly and encoder counts are finite-resolution.

**Scan matching correction.** Taking the odometry prediction as a starting point, SLAM slides the new lidar scan over the existing map to find the pose (x, y, heading) where the scan best overlaps with previously seen features. This is solved by the Ceres nonlinear least-squares optimizer using the Levenberg-Marquardt trust-region strategy. The result is a corrected pose substantially more accurate than raw odometry. The corrected `map → odom` transform is published at 50 Hz for Nav2 to consume.

**Loop closure.** When the robot returns to a previously visited location, the incoming scan suddenly matches a region already in the map. SLAM detects this event and applies a global graph optimisation that pulls all pose estimates into mutual consistency at once — eliminating any drift accumulated during the traversal. Our configuration requires a minimum chain of 10 matching scans before accepting a loop closure, to prevent false positives from symmetrical corridors.

**Output.** The occupancy grid is a 2D image at 5 cm/cell resolution. Each cell is marked occupied (black, probability > 0.65), free (white, probability < 0.25), or unknown (grey). Saved to disk as `cafe_map.pgm` + `cafe_map.yaml` using `ros2 run nav2_map_server map_saver_cli`.

**Key SLAM parameters and their rationale:**

| Parameter | Value | Rationale |
|---|---|---|
| `resolution` | 0.05 m/cell | 5 cm is sufficient for a cafe — finer resolution wastes memory and slows the solver without improving navigation quality |
| `minimum_laser_range` | 0.12 m | Suppresses sub-minimum Gazebo readings published as 0.0 m, which SLAM would incorrectly interpret as hits at the sensor origin, corrupting scan matching |
| `map_update_interval` | 2.0 s | Reduced from default 0.5 s to ease CPU load on the VM; map is re-published every 2 seconds rather than continuously |
| `minimum_travel_distance` | 0.1 m | Robot must move 10 cm before a new scan is processed — prevents redundant solver calls when stationary |
| `minimum_travel_heading` | 0.05 rad | Same purpose for rotation — filters out near-stationary noise |
| `loop_search_maximum_distance` | 3.0 m | Only searches for loop-closure candidates within 3 m, keeping graph search time bounded |
| `transform_publish_period` | 0.02 s | Publishes `map → odom` TF at 50 Hz so Nav2 never waits for a stale transform |

---

### 1.2 AMCL — Monte Carlo Localisation Inside the Saved Map

Once the map is built and saved, SLAM is shut down. For autonomous navigation, the robot must determine its position within that static map in real time as it drives. This is the localisation problem, solved here by AMCL (Adaptive Monte Carlo Localisation).

AMCL maintains a probability distribution over all possible robot poses, represented as a finite set of weighted particles. Each particle is a hypothesis (x, y, heading) about where the robot might be. The algorithm alternates two steps on every sensor update:

**Motion update (predict step).** When the robot moves, all particles are propagated forward using the differential motion model with additive Gaussian noise. Particles spread out slightly, reflecting growing uncertainty. Parameters `alpha1`–`alpha4` (all set to 0.2) control the noise magnitude for each motion type (rotation-from-rotation, rotation-from-translation, translation-from-translation, translation-from-rotation). The `nav2_amcl::DifferentialMotionModel` plugin is used because this robot has no holonomic motion capability.

**Sensor update (weight step).** For each particle, the likelihood field model computes how well the current lidar scan would look if the robot were at that particle's hypothesised pose. Each laser beam's end-point is compared to the nearest occupied cell in the map. Particles in positions where the scan aligns well with map walls receive high weights; particles where the scan misses walls receive low weights.

**Resampling.** Particles with high weights are duplicated; particles with low weights are discarded. Over several update cycles the cloud collapses to a tight cluster at the true robot position.

**Adaptive particle count.** AMCL dynamically varies the number of particles between `min_particles: 500` and `max_particles: 2000` based on the KLD-sampling criterion — more particles when the distribution is spread (high uncertainty), fewer when it is tight (high confidence). This keeps computational cost proportional to the difficulty of the localisation problem.

**Output.** A continuous `map → odom` transform correction broadcast on the TF tree, and a pose-with-covariance on `/amcl_pose`. Nav2's costmap and planner read this transform to express the robot's position in the global map frame.

---

### 1.3 A* Global Planner — Computing the Route

When a navigation goal is sent (counter to table), `nav2_navfn_planner/NavfnPlanner` computes a globally optimal path on the static occupancy grid using A* search.

A* works by expanding cells in order of lowest total estimated cost:

```
f(n) = g(n) + h(n)
```

where `g(n)` is the exact cost from the start to cell `n` (accumulated path distance through the costmap), and `h(n)` is the admissible heuristic — Euclidean straight-line distance from `n` to the goal. Because the heuristic never overestimates the true remaining cost, A* is guaranteed to find the shortest safe path.

**Costmap inflation.** Before planning, every occupied cell in the map is expanded outward by the `inflation_radius` (0.55 m). Cells within this radius are assigned a cost gradient — highest directly adjacent to the obstacle, decreasing with distance. The planner steers through lower-cost regions, naturally keeping the path centred in corridors and away from table edges. This is how the robot avoids table legs whose full physical extent (tabletop + chairs) is larger than the thin pedestal the lidar directly detects.

**Parameters:**

| Parameter | Value | Rationale |
|---|---|---|
| `use_astar` | `true` | A* rather than Dijkstra (see Section 2) |
| `tolerance` | 0.5 m | If the exact goal cell is occupied, accept any cell within 0.5 m — allows navigating to the approximate table location even if a chair partially blocks the exact goal |
| `allow_unknown` | `true` | Allows planning through unexplored grey cells; useful if SLAM left any gaps near table legs |

---

### 1.4 DWB Local Planner — Real-Time Obstacle Avoidance

The global A* path is computed once on the static saved map. It cannot react to a chair that was moved, a customer standing in the aisle, or any object not present when the map was built. DWB (Dynamic Window Based controller, implemented as `dwb_core::DWBLocalPlanner`) handles real-time obstacle avoidance.

On each control cycle (5 Hz in this project, reduced from the default 20 Hz for VM performance), DWB:

1. **Samples velocity commands** — generates a grid of candidate (v_x, ω) pairs over the dynamic window: the range of velocities the robot can physically reach within one control timestep given its acceleration limits (0.5 m/s² linear, 1.5 rad/s² angular).

2. **Simulates trajectories** — for each candidate velocity, simulates the robot's future trajectory forward by `sim_time: 1.5 s` at `linear_granularity: 0.05 m` steps.

3. **Scores trajectories** using multiple critics:
   - **BaseObstacle:** rejects any trajectory that enters a lethal cost cell (hits an obstacle in the local costmap)
   - **PathDist / PathAlign:** rewards trajectories that stay close to and aligned with the global A* path
   - **GoalDist / GoalAlign:** rewards trajectories that progress toward the goal
   - **RotateToGoal:** applies a braking and pure-rotation behaviour when the robot is close to the goal
   - **Oscillation:** penalises back-and-forth motion patterns

4. **Sends the best command** — the velocity with the highest total score is published to `/cmd_vel`. The robot executes it and the cycle repeats.

**Local costmap.** DWB maintains its own 4 m × 4 m rolling window costmap centred on the robot, built from live `/scan` readings. This is separate from the global static costmap. Dynamic obstacles (moving chairs, people) appear as occupied cells in the local costmap and are treated as lethal by the BaseObstacle critic, causing DWB to swerve around them even though they were not in the SLAM map.

---

## 2. Algorithm Selection and Alternatives Considered

### 2.1 A* vs Dijkstra

`nav2_navfn_planner` supports both A* (`use_astar: true`) and Dijkstra (`use_astar: false`). Dijkstra's algorithm is A* with a heuristic of zero — it expands cells in concentric rings from the start point, exploring every direction equally until it reaches the goal.

A* was chosen because it uses the straight-line distance heuristic to bias exploration toward the goal. In a cafe-sized environment with a clear goal direction, A* explores significantly fewer cells than Dijkstra and finds the path faster. The cost in correctness is zero: since Euclidean distance never overestimates the true path cost (the heuristic is admissible), A* finds the same optimal path as Dijkstra but in less time.

Dijkstra would be preferable in environments where the goal direction is unknown or the space is extremely maze-like, since the heuristic provides no benefit and adds overhead when the optimal expansion order is already uniform. For counter-to-table navigation in a structured cafe, A* is the appropriate choice.

### 2.2 EKF / robot_localization — Considered but Deferred

In a full production deployment, wheel odometry and IMU data would be fused using an Extended Kalman Filter (via the `robot_localization` package) before being passed to AMCL. The EKF would:

- **Predict** the robot's state (position, velocity, orientation) using wheel odometry at ~30 Hz
- **Correct** the prediction using IMU angular velocity at 100 Hz, catching yaw drift that encoders cannot detect (e.g., wheel slip on a smooth tile floor)
- Output a fused `/odom` topic with lower covariance than either sensor alone

This fusion would give AMCL a cleaner motion model to work from, meaning fewer particles are needed for reliable convergence and the `map → odom` correction would be more stable.

The decision to defer `robot_localization` for this project was deliberate. In Gazebo Classic simulation, wheel odometry is computed directly from the physics joint states rather than from encoder pulses — there is no mechanical slip and no encoder noise. The diff-drive plugin's `/odom` output is already nearly perfect, so EKF fusion adds implementation complexity without measurable benefit in simulation. The IMU plugin in Gazebo also outputs idealised angular rates with only small Gaussian noise, so AMCL's built-in differential motion model (`alpha1`–`alpha4` = 0.2) adequately represents the odometry uncertainty.

In a physical deployment on real tile floors, EKF fusion would become necessary to handle wheel slip and IMU integration drift. The `ekf.yaml` configuration file has been prepared for this purpose.

### 2.3 DWB vs TEB vs MPPI

Nav2 offers three local planner options: DWB (Dynamic Window Based), TEB (Timed Elastic Band), and MPPI (Model Predictive Path Integral).

**TEB** continuously deforms the global path into an elastic band that minimises travel time while maintaining obstacle clearance. It produces smooth, time-optimal trajectories — preferable for high-speed robots or narrow corridors where precise path-following matters. However, TEB requires the `teb_local_planner` package which is not available in Nav2 Humble's default installation.

**MPPI** samples thousands of random trajectory rollouts using GPU-accelerated Monte Carlo simulation and selects the distribution-weighted optimal command. It handles non-convex obstacle geometry and dynamic environments better than DWB. However, it requires a GPU for real-time performance and is computationally inappropriate for a VM with limited CPU resources.

**DWB** was chosen because it ships with Nav2 Humble, is well-documented, and its critic-based scoring system is directly interpretable — each parameter has a clear physical meaning. At a restaurant robot's conservative maximum speed of 0.3 m/s, the velocity search space is small, making DWB's brute-force sampling approach fast enough even at 5 Hz control frequency.

---

## 3. Key Parameters and Design Trade-offs

### 3.1 Costmap Inflation Radius — 0.55 m

The lidar physically detects the table pedestals (cylinders of 0.06 m radius, extended to 1.5 m height to reach the lidar scan plane at z ≈ 1.357 m). However, the tables themselves have a radius of approximately 0.4 m and each table has four chairs extending an additional 0.3–0.4 m outward. The robot body has a half-width of 0.23 m.

Setting `inflation_radius: 0.55 m` means the costmap inflates each pedestal detection outward by 0.55 m. This places the high-cost zone at approximately 0.06 + 0.55 = 0.61 m from the pedestal centre, which approximates the outer edge of the chair footprint. The robot's own body then requires clearance equal to its half-width (0.23 m) from the edge of the inflated zone — giving a total effective table-plus-chairs exclusion zone of approximately 0.61 + 0.23 = 0.84 m from the pedestal centre.

Aisle clearance verification: the tables are positioned at x = ±3.5 m. The central aisle between them is therefore (3.5 - 3.5) - (2 × 0.84) = 7.0 m - 1.68 m = 5.32 m wide — comfortably navigable. The narrowest passage (between a table chair and the side wall at x = ±6 m) is approximately 6.0 - 3.5 - 0.84 = 1.66 m, which is sufficient for a 0.46 m wide robot.

A larger inflation radius (e.g., 0.8 m) would block aisles near the walls. A smaller one (e.g., 0.3 m) would allow the planner to route the robot through chair legs, causing physical collisions.

### 3.2 Controller and Costmap Frequencies — VM Trade-offs

Default Nav2 frequencies are tuned for real hardware or high-performance workstations. The VM hosting this project runs at approximately 52% real-time speed (confirmed from Gazebo clock output). All frequency parameters were reduced to maintain stability:

| Parameter | Default | This Project | Reason |
|---|---|---|---|
| `controller_frequency` | 20 Hz | 5 Hz | DWB trajectory sampling takes ~50 ms on the VM; 20 Hz would cause each iteration to run over budget and produce control lag |
| `local_costmap update_frequency` | 5 Hz | 3 Hz | Reduces scan-to-costmap update load; local obstacles still updated fast enough for 0.3 m/s robot speed |
| `global_costmap update_frequency` | 1 Hz | 1 Hz | Static map rarely changes; 1 Hz is sufficient |
| `lidar update_rate` | 10 Hz | 5 Hz | Halved to reduce the volume of scan messages competing for CPU with the Nav2 stack |
| `map_update_interval` (SLAM) | 0.5 s | 2.0 s | SLAM solver runs less frequently, reducing peak CPU contention during mapping |

On real hardware these reductions would not be necessary. The robot's top speed was also set conservatively at `max_vel_x: 0.3 m/s` (versus a typical 0.8–1.0 m/s for a restaurant robot) to keep DWB's dynamic window small and trajectory scoring tractable at 5 Hz.

### 3.3 Robot Footprint

The robot chassis is 0.46 m × 0.46 m. The footprint is specified as a square polygon:

```yaml
footprint: "[[-0.23, -0.23], [-0.23, 0.23], [0.23, 0.23], [0.23, -0.23]]"
```

This is used by both the local and global costmap for collision checking. A circular approximation (`robot_radius`) would be simpler to configure but less accurate for a square chassis — it would either under-represent the corners (allowing corner collisions) or over-represent the sides (unnecessarily restricting passage width). The polygon representation correctly models all four corners.

### 3.4 Goal Tolerances

```yaml
xy_goal_tolerance: 0.25 m
yaw_goal_tolerance: 0.25 rad (≈14°)
```

These were set larger than the Nav2 defaults (0.05 m, 0.05 rad) for two reasons. First, at 5 Hz control frequency the robot cannot correct position at fine resolution before the goal-check fires. Second, for a delivery robot stopping at a table, a 25 cm positional tolerance is operationally acceptable — the robot is close enough to the table to serve food without needing sub-centimetre precision.

---

## 4. Major Errors Encountered and Debugging

### Error 1 — SLAM White Map (Complete Mapping Failure)

**Symptom.** After manually driving the full cafe perimeter, `cafe_map.pgm` was entirely white — no occupied cells despite clearly driving past walls, tables, and the counter.

**Root cause 1 — Minimum laser range.** The Gazebo lidar plugin publishes a range of exactly 0.0 m for any reading below the sensor's physical minimum range (0.12 m). SLAM Toolbox with default settings (`minimum_laser_range: 0.0`) accepted these 0.0 m readings as genuine hits at the sensor origin. This corrupted scan matching, because every scan appeared to contain hundreds of obstacles clustered at the robot's centre — making matching with the actual environment impossible.

**Fix.** Added `minimum_laser_range: 0.12` to `slam_params.yaml`, matching the URDF sensor `<min>` range. This caused SLAM Toolbox to silently discard all sub-minimum readings before attempting to match them.

**Root cause 2 — Obstacles below the lidar scan plane.** The lidar is mounted at z = 1.307 m on the robot body. With the robot on the ground, the scan plane is at approximately z = 1.357 m in world coordinates. The original cafe world modelled tables with tops at z = 0.76 m, pedestals at 0.74 m, chairs at z = 0.9 m, and the service counter at z = 1.0 m — all below the scan plane. The lidar produced only 360° of maximum-range readings (12 m) because there was physically nothing for it to hit.

**Fix.** Extended all six table pedestals from 0.74 m to 1.5 m height, and the service counter from 1.0 m to 1.5 m height. At 1.5 m, all obstacles intersect the scan plane at z ≈ 1.357 m, making them detectable. Confirmed with `Non-max readings: 338/360` — 338 of 360 laser rays were returning genuine hits from walls and pedestals.

**Root cause 3 — VM async queue drops.** Even after fixing the above, the map still showed fragmentary walls on two sides and nothing on the other two. Investigation revealed the VM was running at ~52% real-time speed. SLAM Toolbox's internal async processing queue was silently dropping most incoming scans — only processing scans when the solver thread was free. At 52% speed, the solver frequently fell behind. The result was that scans were being processed only at a sparse, irregular subset of robot positions — insufficient for wall coverage.

**Fix.** Three parallel mitigations: (a) disabled Gazebo's rendering GUI (`gui: false`), the largest single CPU saving; (b) reduced lidar update rate from 10 Hz to 5 Hz, halving the queue arrival rate; (c) increased `map_update_interval` from 0.5 s to 2.0 s, allowing the solver more time between map publishes. After these changes, all four walls and all six pedestals mapped correctly.

---

### Error 2 — Nav2 Crash: Wrong Plugin Format (/ vs ::)

**Symptom.** Nav2 failed to start with `failed to create behavior spin of type nav2_behaviors::Spin ... class nav2_behaviors::spin does not exist, declared types are nav2_behaviors/spin`.

**Root cause.** Nav2 uses two different plugin name formats depending on the plugin category, and the distinction is not documented in one place:
- **`/` format** (pluginlib class loader format): `nav2_behaviors/Spin`, `nav2_behaviors/BackUp`, `nav2_navfn_planner/NavfnPlanner`, `nav2_bt_navigator/NavigateToPoseNavigator`
- **`::` format** (C++ namespace format): `nav2_amcl::DifferentialMotionModel`, `nav2_costmap_2d::ObstacleLayer`, `nav2_costmap_2d::InflationLayer`, `dwb_core::DWBLocalPlanner`, `nav2_controller::SimpleGoalChecker`

**Fix.** Read the plugin XML registration files from `/opt/ros/humble/share/` for each package and compared against the official `nav2_bringup` parameter file. Corrected each plugin string to use the format registered in its respective `plugin.xml`. The general rule (not stated in Nav2 docs): packages that register as ROS pluginlib plugins with an explicit class name use `/`; packages that expose their class via C++ `PLUGINLIB_EXPORT_CLASS` macro use `::`.

---

### Error 3 — RViz "No Map Received"

**Symptom.** RViz Map display showed "No map received" despite Nav2 running and `/map` appearing in `ros2 topic list`.

**Root cause.** `map_server` publishes `/map` with QoS durability set to `TRANSIENT_LOCAL` — meaning it retains the last published message for any late-joining subscriber. RViz's Map display subscribes with the default `VOLATILE` durability. Under ROS 2's QoS compatibility rules, a `VOLATILE` subscriber cannot receive messages from a `TRANSIENT_LOCAL` publisher. The map was being published but RViz was invisible to it.

**Fix.** In RViz's Map display, expanded the Topic section and changed Durability Policy from `Volatile` to `Transient Local`. The map appeared immediately.

---

### Error 4 — Laser Scan Misaligned in Map Frame

**Symptom.** After placing the 2D Pose Estimate in RViz, the laser scan appeared as a long diagonal line offset from the robot rather than tracing nearby walls. The costmap filled entirely with obstacles as a result.

**Diagnosis.** Switched RViz Fixed Frame from `map` to `odom` — the scan immediately aligned correctly with the robot's surroundings. This confirmed the `odom → lidar_link` transform chain was correct. The fault was specifically in the `map → odom` transform published by AMCL — the pose estimate was wrong.

**Root cause.** AMCL's `set_initial_pose` parameter was set to `x: 0.0, y: -1.0` (the intended spawn position). However, the robot's actual position in the map frame was `x: -5.57, y: -1.08` — the robot had been displaced from the spawn point by Gazebo physics, ending up against the west wall. AMCL therefore initialised its particle cloud at the wrong location, estimated a `map → odom` transform that was 5.57 m off in the x-axis, and every scan point was projected 5.57 m away from its real position in the map frame.

**Fix.** Used `ros2 run tf2_ros tf2_echo map base_link` to read the robot's actual map-frame position directly from the TF tree. Updated `nav2_params.yaml` `initial_pose.x` to `-5.64` to match. Used the 2D Pose Estimate tool in RViz to click at the robot's true location and drag the arrow to match its heading. After the robot moved approximately 0.25 m (the `update_min_d` threshold), AMCL's particle filter converged and the scan snapped to the wall positions.

---

### Error 5 — AMCL Active but Particle Cloud Not Publishing

**Symptom.** `ros2 lifecycle get /amcl` returned `active[3]`, but `ros2 topic hz /particle_cloud` reported the topic was not being published.

**Root cause.** AMCL activated before `map_server` had finished loading and publishing the map. AMCL's initialisation sequence requires the map to be available to set up the likelihood field. When the map was not yet available at activation time, AMCL silently failed to initialise its particle filter and produced no output, despite reporting itself as active.

Additionally, in Nav2 Humble, AMCL publishes to `/particle_cloud` with type `nav2_msgs/msg/ParticleCloud`, while RViz's PoseArray display subscribes to `geometry_msgs/msg/PoseArray` — a type mismatch that prevented visualisation even when the topic was active.

**Fix.** Manually published the initial pose via `ros2 topic pub /initialpose` to force AMCL to reinitialise its particle filter after the map was confirmed available. For visualisation, added the `nav2_rviz_plugins/ParticleCloud` display type instead of the standard PoseArray display.

---

## 5. Supporting Evidence — Counter-to-Table Navigation

The following evidence supports the central deliverable of successful counter-to-table navigation with obstacle avoidance:

### What to capture for your report:

**Screenshot 1 — Saved SLAM map.**
Open `cafe_map.pgm` in any image viewer. Show the occupancy grid with black walls, white free space, and the six pedestal points marking table positions.

**Screenshot 2 — AMCL localisation in RViz.**
With Nav2 running, take a screenshot showing:
- The cafe map (grey/white/black)
- The laser scan points (red dots) aligned with the map wall edges
- The robot model at its current position

**Screenshot 3 — A* global path.**
In RViz, send a 2D Nav Goal from the counter area (y ≈ -10, x ≈ 0) to a table position (e.g., x = 3.5, y = -2.5). Take a screenshot showing the green planned path curving around the inflated obstacle zones.

**Screenshot 4 — DWB costmap.**
Add the Local Costmap display in RViz (topic `/local_costmap/costmap`). Take a screenshot showing the red/orange inflation zones around nearby table pedestals and walls, with the robot positioned in a clear corridor.

**Screenshot 5 — Goal reached.**
Take a screenshot after the robot successfully reaches the table goal, showing the robot model at the destination with the scan correctly tracing the surrounding environment.

**Recording (optional).** Use `ros2 bag record /scan /odom /amcl_pose /plan /cmd_vel` to capture a full counter-to-table run. Screen-record RViz during the run for a visual demonstration.

---

## Appendix: Key File Reference

| File | Purpose |
|---|---|
| `urdf/serv_robot.urdf` | Robot description: diff-drive plugin, lidar at z=1.307 m, wheel geometry |
| `worlds/cafe.world` | Cafe environment: 6 tables with 1.5 m pedestals, counter at 1.5 m |
| `config/slam_params.yaml` | SLAM Toolbox configuration: minimum_laser_range, VM-tuned frequencies |
| `config/nav2_params.yaml` | Full Nav2 stack: AMCL, A*, DWB, costmaps, initial pose |
| `maps/cafe_map.yaml` | Saved map metadata: origin, resolution, occupancy thresholds |
| `launch/gazebo.launch.py` | Launches Gazebo headless + robot_state_publisher + entity spawn |
| `launch/nav2.launch.py` | Launches full Nav2 bringup with cafe map and nav2_params.yaml |
