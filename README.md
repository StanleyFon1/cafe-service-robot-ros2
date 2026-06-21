# serv_robot — Autonomous Restaurant Service Robot

ROS 2 Humble + Gazebo Classic 11 simulation of a differential-drive robot that navigates autonomously from a service counter to customer tables in a cafe environment.

## Dependencies

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select serv_robot --symlink-install
source install/setup.bash
```

## Launch

### Step 1 — Start Gazebo

```bash
ros2 launch serv_robot gazebo.launch.py
```

The robot spawns automatically after 15 seconds. Confirm it is live before proceeding:
```bash
ros2 topic hz /scan   # should show ~5 Hz
```

> **Slow machine?** If the spawn fails (exit code 1 in the terminal), run it manually:
> ```bash
> ros2 run gazebo_ros spawn_entity.py -entity serv_robot -topic /robot_description -x 0 -y -5 -z 0.05 -Y 0
> ```

### Step 2 — Start Nav2 (uses pre-built map)

```bash
ros2 launch serv_robot nav2.launch.py
```

### Step 4 — Visualise in RViz

```bash
ros2 run rviz2 rviz2 --display-config $(ros2 pkg prefix serv_robot)/share/serv_robot/config/nav2.rviz
```

The config loads automatically with the correct Fixed Frame (`map`), Map (Transient Local QoS), LaserScan, RobotModel, and global path display pre-configured. The laser scan should align with the map walls automatically (AMCL auto-initialises at spawn position).

### Step 5 — Send a navigation goal

Click **2D Nav Goal** in the RViz toolbar, click a point on the open floor, drag to set heading.

Table positions for delivery goals:

| Table | x | y |
|-------|---|---|
| 1 (left front) | -3.5 | -2.5 |
| 2 (right front) | 3.5 | -2.5 |
| 3 (left middle) | -3.5 | -5.5 |
| 4 (right middle) | 3.5 | -5.5 |
| 5 (left back) | -3.5 | -8.5 |
| 6 (right back) | 3.5 | -8.5 |
| Counter | 0 | -10.4 |

---

## Rebuilding the Map (optional)

Only needed if the cafe layout changes.

```bash
# Terminal 1 — Gazebo + robot (Steps 1 & 2 above)

# Terminal 2 — SLAM
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/ros2_ws/src/serv_robot/config/slam_params.yaml \
  use_sim_time:=true

# Terminal 3 — Drive the cafe
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 — Save map when complete
ros2 run nav2_map_server map_saver_cli \
  -f $HOME/ros2_ws/src/serv_robot/maps/cafe_map
```

---

## Package Structure

```
serv_robot/
├── config/
│   ├── nav2_params.yaml      # AMCL, A*, DWB, costmap parameters
│   └── slam_params.yaml      # SLAM Toolbox parameters
├── launch/
│   ├── gazebo.launch.py      # Gazebo + robot_state_publisher
│   └── nav2.launch.py        # Full Nav2 stack with pre-built map
├── maps/
│   ├── cafe_map.pgm          # Occupancy grid (56 KB)
│   └── cafe_map.yaml         # Map metadata (origin, resolution)
├── meshes/                   # SolidWorks STL exports
├── urdf/
│   └── serv_robot.urdf       # Robot description with Gazebo plugins
├── worlds/
│   └── cafe.world            # Cafe SDF (walls, tables, chairs, counter)
└── REPORT_DRAFT.md           # Full technical report
```

## Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Wheel separation | 0.425 m | Diff-drive odometry |
| Wheel diameter | 0.162 m | Diff-drive odometry |
| Lidar height | 1.307 m | Above table tops — detects pedestals & chairs |
| Inflation radius | 0.85 m | Covers table pedestal + full chair footprint |
| Max velocity | 0.3 m/s | Conservative speed for indoor delivery |
| Map resolution | 0.05 m/cell | 5 cm grid |
