# serv_robot — Autonomous Café Service Robot

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-FF7300)
![Nav2](https://img.shields.io/badge/Navigation-Nav2-2F6FB0)
![License](https://img.shields.io/badge/License-MIT-2E8B57)

A **ROS 2 Humble + Gazebo Classic 11** simulation of a differential-drive service robot that navigates autonomously from a service counter to customer tables in a café environment. The robot localizes on a pre-built map, plans collision-free paths around tables and chairs with the Nav2 stack, and drives to semantic table goals.

> Developed by **Team Bumblebee** as a case study for *Cooperating and Autonomous Systems*, Deggendorf Institute of Technology (DIT/THD).

<!-- Add a screenshot or GIF of the robot navigating the café here — it makes the repo. -->
<!-- ![Café robot navigating in RViz](docs/demo.png) -->

---

## Features

- **Differential-drive robot** modelled in SolidWorks and exported to URDF with Gazebo plugins.
- **Custom café world** — enclosed room with tables, chairs, and a service counter.
- **SLAM mapping** with `slam_toolbox` and a saved occupancy grid.
- **Autonomous navigation** — AMCL localization, A\* global planning, and the DWB local controller over layered costmaps.
- **Semantic delivery goals** — drive to named tables via RViz or an automated mission node.

## Tech stack

`ROS 2 Humble` · `Gazebo Classic 11` · `Nav2` · `slam_toolbox` · `AMCL` · `Python (rclpy)` · `Ubuntu 22.04`

---

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
# clone the repository into your workspace
git clone https://github.com/StanleyFon1/cafe-service-robot-ros2.git ~/serv_bot
cd ~/serv_bot

colcon build --packages-select serv_robot --symlink-install
source install/setup.bash
```

> Remember to `source ~/serv_bot/install/setup.bash` in every new terminal before running the commands below.

---

## Running the Simulation

### Step 1 — Start Gazebo

```bash
ros2 launch serv_robot gazebo.launch.py
```

The robot spawns automatically after ~15 seconds. Confirm it is live before continuing:

```bash
ros2 topic hz /scan   # should report ~5 Hz
```

> **Slow machine?** If the automatic spawn fails (exit code 1 in the terminal), spawn the robot manually:
> ```bash
> ros2 run gazebo_ros spawn_entity.py \
>   -entity serv_robot -topic /robot_description \
>   -x 0 -y -5 -z 0.05 -Y 0
> ```

### Step 2 — Start Nav2 (uses the pre-built map)

```bash
ros2 launch serv_robot nav2.launch.py
```

### Step 3 — Visualise in RViz

```bash
ros2 run rviz2 rviz2 --display-config \
  $(ros2 pkg prefix serv_robot)/share/serv_robot/config/nav2.rviz
```

The configuration loads automatically with the correct Fixed Frame (`map`), Map (Transient Local QoS), LaserScan, RobotModel, and global path display pre-configured. The laser scan aligns with the map walls automatically, since AMCL auto-initialises at the spawn position.

### Step 4 — Send a navigation goal

In the RViz toolbar, click **2D Nav Goal**, click a point on the open floor, and drag to set the heading. Use the table coordinates below as delivery targets.

| Table            |    x |     y |
|------------------|-----:|------:|
| 1 (left front)   | -3.5 |  -2.5 |
| 2 (right front)  |  3.5 |  -2.5 |
| 3 (left middle)  | -3.5 |  -5.5 |
| 4 (right middle) |  3.5 |  -5.5 |
| 5 (left back)    | -3.5 |  -8.5 |
| 6 (right back)   |  3.5 |  -8.5 |
| Counter          |  0.0 | -10.4 |

### Step 5 — Run the automated delivery mission (optional)

Instead of sending goals manually, run the delivery node from the workspace root to execute a full multi-table mission, with a timed serving wait at each table:

```bash
cd ~/serv_bot
python3 delivery_node.py
```

---

## Rebuilding the Map (optional)

Only required if the café layout changes.

```bash
# Terminal 1 — Gazebo + robot (Steps 1 & 2 above)

# Terminal 2 — SLAM
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/serv_bot/config/slam_params.yaml \
  use_sim_time:=true

# Terminal 3 — Drive the café manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 — Save the map when coverage is complete
ros2 run nav2_map_server map_saver_cli \
  -f $HOME/serv_bot/maps/cafe_map
```

---

## Project Structure

```
~/serv_bot/                   # workspace (package files at the root)
├── config/
│   ├── nav2_params.yaml      # AMCL, A*, DWB, and costmap parameters
│   └── slam_params.yaml      # SLAM Toolbox parameters
├── launch/
│   ├── gazebo.launch.py      # Gazebo + robot_state_publisher
│   └── nav2.launch.py        # Full Nav2 stack with the pre-built map
├── maps/
│   ├── cafe_map.pgm          # Occupancy grid
│   └── cafe_map.yaml         # Map metadata (origin, resolution)
├── meshes/                   # SolidWorks STL exports
├── urdf/
│   └── serv_robot.urdf       # Robot description with Gazebo plugins
├── worlds/
│   └── cafe.world            # Café SDF (walls, tables, chairs, counter)
├── delivery_node.py          # Autonomous delivery mission (NavigateToPose client)
├── CMakeLists.txt
├── package.xml
├── LICENSE
└── README.md
```

## Key Parameters

| Parameter         | Value        | Purpose                                            |
|-------------------|--------------|----------------------------------------------------|
| Wheel separation  | 0.425 m      | Differential-drive odometry                        |
| Wheel diameter    | 0.162 m      | Differential-drive odometry                        |
| LiDAR height      | 1.307 m      | Above table tops — detects pedestals and chairs    |
| Inflation radius  | 0.85 m       | Covers the table pedestal and full chair footprint |
| Max velocity      | 0.3 m/s      | Conservative speed for indoor delivery             |
| Map resolution    | 0.05 m/cell  | 5 cm grid                                          |

---

## Team

**Team Bumblebee** — Deggendorf Institute of Technology (DIT/THD)

- Stanley Fon
- Lara Ipek
- Abdul Aziz Abbas
- Muhammed Umar Ansari

Course: *Case Studies: Cooperating and Autonomous Systems*

## License

Released under the [MIT License](LICENSE).
