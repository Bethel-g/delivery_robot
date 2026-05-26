# Autonomous Indoor Delivery Robot (PROJECT BY BETHEL, ESROM AND YANIT — AAIT DEPARTMENT OF AI)

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange)](https://classic.gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-latest-green)](https://navigation.ros.org/)
[![slam_toolbox](https://img.shields.io/badge/slam__toolbox-latest-blueviolet)](https://github.com/SteveMacenski/slam_toolbox)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?logo=ubuntu)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Build](https://img.shields.io/badge/build-colcon-brightgreen)]()

> A **fully simulated** autonomous indoor delivery robot built with industry-standard tools.
> The robot explores an unknown office, builds its own map in real time, then accepts delivery
> commands and navigates autonomously — no hardware required.
> Compares two navigation algorithms: **DWA** and **MPPI**.

---

## Demo

> *Record your own demo: `kazam` or `peek` for screen capture → convert to GIF → save as `docs/demo.gif`*

```
Phase 1 — SLAM:        Robot drives around office, building occupancy grid map
Phase 2 — Navigation:  "room1 room3 room2" → robot picks up package, delivers to each room, returns to base
```

---

## Features

| Feature | Details |
|---|---|
| **Custom Robot Model** | Differential-drive robot (URDF/Xacro) with 360° 2D LIDAR, IMU, caster wheel, and realistic inertia tensors |
| **Realistic Indoor World** | 10×8 m multi-room Gazebo office: 4 rooms, corridor, doors, desks, filing cabinets, two dynamic obstacles |
| **Real-Time SLAM** | `slam_toolbox` online-async mapping — robot builds occupancy grid without any prior map |
| **Dual Algorithm Comparison** | Algorithm 1: NavFn A\* + DWB — Algorithm 2: Smac Hybrid A\* + MPPI — selectable at launch |
| **Full Nav2 Stack** | AMCL localisation · global planner · local controller · costmaps · behaviour-tree navigator |
| **Delivery Mission Node** | State machine: IDLE→PICKUP→LOADED→NAVIGATING→DELIVERING→RETURNING with green RViz payload marker |
| **Dynamic Obstacles** | Two moving boxes (orange circular, blue linear) controlled via `/gazebo/set_entity_state` |
| **Metrics Logger** | Records duration, path length, average speed, and recovery count to `~/delivery_metrics.csv` |
| **Waypoint Recorder** | Interactive tool to record room coordinates from actual robot poses after mapping |
| **Health Checker** | Pre-flight node that verifies all required topics and the Nav2 action server are live |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        User / Terminal                               │
│   ros2 run delivery_robot delivery_mission room1 room3 room2        │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ NavigateToPose action
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                          Nav2 Stack                                   │
│                                                                       │
│  algorithm:=dwa  → NavFn A* global planner  + DWB local controller  │
│  algorithm:=mppi → Smac Hybrid A* planner   + MPPI local controller │
│                                                                       │
│  BT Navigator → Global Planner → Local Controller → /cmd_vel        │
│  ↕ costmaps         ↕ /plan          ↕ velocity_smoother            │
│  AMCL Localiser ←── /scan (LIDAR)    collision_monitor              │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ /cmd_vel, /odom, /scan, TF
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                       Gazebo Simulation                               │
│  URDF Robot ← diff_drive plugin ← libgazebo_ros_state               │
│  LIDAR plugin → /scan           IMU plugin → /imu                   │
│  dynamic_box (orange, circular) + dynamic_box2 (blue, linear)       │
│  delivery_item (green package) — hidden on pickup, placed on drop   │
└──────────────────────────────────────────────────────────────────────┘
```

### ROS 2 Topic Map

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → Nav2/SLAM | 360° LIDAR readings |
| `/odom` | `nav_msgs/Odometry` | Gazebo → Nav2 | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → Gazebo | Velocity commands |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM/map_server → Nav2 | Occupancy grid |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL → Nav2 | Localised robot pose |
| `/plan` | `nav_msgs/Path` | Nav2 → RViz | Global planned path |
| `/delivery_status` | `std_msgs/String` | Mission → Any | Mission event log |
| `/payload_marker` | `visualization_msgs/Marker` | Mission → RViz | Green box on robot when loaded |

---

## Package Structure

```
delivery_robot/
├── delivery_robot/              # Python nodes
│   ├── __init__.py
│   ├── delivery_mission.py      # Main mission node — state machine + NavigateToPose client
│   ├── dynamic_obstacles.py     # Moves two obstacle boxes in Gazebo world
│   ├── metrics_logger.py        # Records performance CSV per mission
│   ├── record_waypoints.py      # Interactive waypoint recorder
│   └── health_check.py          # Pre-flight system check
│
├── urdf/
│   └── robot.urdf.xacro         # Robot model: chassis, wheels, LIDAR, IMU
│
├── worlds/
│   └── office.world             # 4-room office with dynamic obstacles and delivery_item
│
├── maps/
│   └── office_map.yaml          # Map metadata (office_map.pgm is generated by SLAM, not tracked)
│
├── config/
│   ├── nav2_params_dwa.yaml     # Algorithm 1: NavFn A* + DWB
│   ├── nav2_params_mppi.yaml    # Algorithm 2: Smac Hybrid A* + MPPI
│   └── slam_params.yaml         # slam_toolbox configuration
│
├── launch/
│   ├── slam_launch.py           # Phase 1: Gazebo + robot + SLAM + RViz
│   └── navigation_launch.py     # Phase 2: Gazebo + robot + Nav2 + dynamic obstacles + RViz
│
├── rviz/
│   ├── slam.rviz                # RViz config for SLAM
│   └── navigation.rviz          # RViz config for navigation
│
├── test/
│   └── test_delivery_mission.py # Unit tests (pytest)
│
├── package.xml
├── setup.py
└── README.md
```

---

## Quick Start

### Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble** installed ([official instructions](https://docs.ros.org/en/humble/Installation.html))
- **~4 GB free disk space** for ROS 2 + Gazebo

### 1. Install dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-rosdep

sudo rosdep init && rosdep update
```

### 2. Clone and build

```bash
mkdir -p ~/delivery_ws/src
cd ~/delivery_ws/src
git clone https://github.com/Esraprojects/delivery_robot.git

cd ~/delivery_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source /opt/ros/humble/setup.bash
source ~/delivery_ws/install/setup.bash
```

### 3. Phase 1 — Map the environment (SLAM)

```bash
# Terminal 1: Launch SLAM + Gazebo + RViz
ros2 launch delivery_robot slam_launch.py

# Terminal 2: Drive the robot through all rooms to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: When the map looks complete, save it
ros2 run nav2_map_server map_saver_cli \
  -f ~/delivery_ws/src/delivery_robot/maps/office_map
```

Watch the occupancy grid grow in RViz2 as you drive through each room and corridor.

### 4. Phase 2 — Autonomous delivery

```bash
# Terminal 1: Launch navigation stack — choose your algorithm
ros2 launch delivery_robot navigation_launch.py algorithm:=dwa
# or
ros2 launch delivery_robot navigation_launch.py algorithm:=mppi

# Wait ~15 seconds for Gazebo, Nav2, dynamic obstacles, and RViz2 to fully start

# Terminal 2: Run the delivery mission
ros2 run delivery_robot delivery_mission room1 room3 room2

# Terminal 3 (optional): Watch live mission events
ros2 topic echo /delivery_status
```

The robot will: load the package at base → navigate to each room → deliver → return to base.
A green box appears in RViz2 while the robot is carrying the package.

### 5. View performance metrics

```bash
cat ~/delivery_metrics.csv
```

Run once with `algorithm:=dwa` and once with `algorithm:=mppi` to compare results.

---

## Mission Node Reference

```bash
# Basic delivery route (returns to base automatically)
ros2 run delivery_robot delivery_mission room1 room2

# Multi-stop route
ros2 run delivery_robot delivery_mission room1 room3 room4 room2

# Run with MPPI algorithm label (for metrics correlation)
ros2 run delivery_robot delivery_mission --algorithm mppi room1 room2

# Loop mode — repeat indefinitely (Ctrl-C to stop)
ros2 run delivery_robot delivery_mission --loop room1 room2

# Skip return-to-base
ros2 run delivery_robot delivery_mission --no-return room1

# List all known rooms and coordinates
ros2 run delivery_robot delivery_mission --list
```

Available rooms: `base`, `room1`, `room2`, `room3`, `room4`, `corridor_left`, `corridor_right`

---

## Core Concepts

### SLAM — Simultaneous Localisation and Mapping

The robot starts in an unknown environment with no prior map. `slam_toolbox` simultaneously estimates the robot's pose and builds the map by:

1. Matching new LIDAR scans to previously seen geometry (scan matching)
2. Detecting when the robot revisits a location (loop closure)
3. Optimising the entire pose graph to remove accumulated drift

**Result:** A metric occupancy grid where each cell is `free` (white), `occupied` (black), or `unknown` (grey).

### Nav2 Navigation Stack

| Component | Role |
|---|---|
| **map_server** | Serves the saved `.pgm` occupancy grid as a ROS topic |
| **AMCL** | Particle filter: localises the robot within the saved map using live LIDAR |
| **Global Costmap** | Inflated static map — plans around known walls with a safety margin |
| **Local Costmap** | Rolling window around the robot — avoids dynamic obstacles in real time |
| **NavFn A\* / Smac Hybrid A\*** | Finds globally optimal collision-free path (DWA mode / MPPI mode) |
| **DWB / MPPI** | Converts the global path to safe velocity commands (DWA mode / MPPI mode) |
| **BT Navigator** | Orchestrates the above via a Behaviour Tree — handles retries and recovery |

### TF2 Transform Tree

```
map
 └─ odom          (published by AMCL — corrects for drift)
     └─ base_footprint   (published by diff_drive plugin)
         └─ base_link
             ├─ left_wheel
             ├─ right_wheel
             ├─ caster_link
             ├─ laser_link   (LIDAR sensor)
             └─ imu_link
```

---

## Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| Robot not moving | `/cmd_vel` not reaching Gazebo | Check `ros2 topic echo /cmd_vel` and diff_drive plugin name |
| Map not building | LIDAR topic mismatch | Verify `scan_topic: /scan` in `slam_params.yaml` |
| AMCL not converging | Wrong initial pose | Use RViz2 "2D Pose Estimate" tool to set initial pose manually |
| Nav2 goal rejected | Costmap inflation blocking goal | Increase `xy_goal_tolerance` or move waypoint away from walls |
| Robot oscillates | Controller tuning | Reduce `PathAlign.scale` (DWB) or adjust `vx_max` (MPPI) |
| Dynamic obstacles not moving | Gazebo state service not ready | Wait the full 15 s startup delay; check `/gazebo/set_entity_state` |
| Gazebo crashes | GPU memory (WSL2) | Set `LIBGL_ALWAYS_SOFTWARE=1 gazebo ...` for software rendering |

---

## Running Tests

```bash
cd ~/delivery_ws
colcon test --packages-select delivery_robot
colcon test-result --verbose

# Or run pytest directly
cd src/delivery_robot
python3 -m pytest test/ -v
```

---

## Team Roles

| Member | Role | Key Files |
|---|---|---|
| **Robot Designer** | URDF model, sensor parameters, Gazebo physics tuning | `urdf/robot.urdf.xacro`, `worlds/office.world` |
| **Navigation Lead** | SLAM config, Nav2 parameter tuning, algorithm comparison | `config/nav2_params_dwa.yaml`, `config/nav2_params_mppi.yaml` |
| **Mission Developer** | Delivery node, dynamic obstacles, metrics logger | `delivery_robot/*.py` |
| **Integration Lead** | Launch files, Git workflow, demo, documentation | `launch/*.py`, `README.md` |

---

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo Classic Tutorials](https://classic.gazebosim.org/tutorials)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [DWB Local Planner](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [MPPI Controller](https://navigation.ros.org/configuration/packages/configuring-mppic.html)
- [AMCL Configuration](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

---

## License

MIT License — see [LICENSE](LICENSE) for details.
