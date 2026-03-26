# maze_solver_bot 🧩

An autonomous maze-solving robot simulation built with **ROS2 Jazzy** and **Gazebo Harmonic**, featuring frontier-based exploration, EKF-fused localization, wheel slip detection, and full Nav2 integration.

---

## 📸 Demo

<p align="center">
  <img src="Demos/img1.png" width="48%" />
  <img src="Demos/img2.png" width="48%" />
</p>

> 🎥 Full exploration demo: [`Demos/DemoExploration.mp4`](Demos/DemoExploration.mp4)

---

## ✨ Features

- 🗺️ **Frontier-based autonomous exploration** — robot maps and solves the maze without any prior knowledge
- 🔬 **EKF sensor fusion** — wheel odometry + IMU fused via `robot_localization` for accurate pose estimation
- 🛡️ **LiDAR-based wheel slip detection** — corrects odometry when the robot is stuck against a wall
- 🧭 **SLAM Toolbox** integration with tuned parameters for tight indoor environments
- ⚡ **Nav2** stack with MPPI controller for smooth, fast path following
- 🎮 **Manual teleoperation** via keyboard or joystick
- 📡 **RViz2** visualization with custom config

---

## 🛠️ Prerequisites

| Dependency | Version |
|---|---|
| ROS2 | Jazzy |
| Gazebo | Harmonic |
| Nav2 | ros-jazzy-nav2-bringup |
| robot_localization | ros-jazzy-robot-localization |
| slam_toolbox | ros-jazzy-slam-toolbox |
| teleop_twist_joy | Optional – joystick control |

Install dependencies:
```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
```

---

## 📦 Building

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/adios-07/maze_solver_bot.git
cd ~/ros2_ws
colcon build --packages-select maze_solver_bot
source install/setup.bash
```

---

## 🚀 Launch Modes

### Mode 1 — Simulation (Manual Control)

Spawns the robot in Gazebo Harmonic with SLAM Toolbox and RViz2. Drive the robot manually to build a map.

```bash
ros2 launch maze_solver_bot launch_sim.launch.py
```

**Keyboard control** (in a new terminal):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Joystick control:**
```bash
# Joy node runs by default — configure button mappings in:
config/joystick.yaml
```

---

### Mode 2 — Autonomous Exploration

Spawns the robot and runs the full autonomous exploration pipeline. The robot will map the entire maze by itself using frontier-based exploration.

```bash
ros2 launch maze_solver_bot launch_sim.launch.py
```

Then in a new terminal:
```bash
ros2 run maze_solver_bot frontier_explorer.py
```

The explorer will:
1. Wait for the SLAM map and Nav2 to initialize
2. Detect frontier boundaries between known and unknown space
3. Navigate to the highest-value frontier (largest + closest)
4. Repeat until the maze is fully mapped

---

### Mode 3 — Bringup (Autonomous Navigation on Saved Map)

Load a previously saved map and navigate autonomously to goal poses.

```bash
ros2 launch maze_solver_bot navigation_launch.py
```

**To navigate:**
1. In RViz2, click **2D Pose Estimate** → set the robot's starting position
2. Click **Nav2 Goal** → send the robot to a target location

---

## 📁 Project Structure

```
maze_solver_bot/
├── config/
│   ├── diff_drive_controller.yaml      # ros2_control controller config
│   ├── ekf.yaml                        # Extended Kalman Filter params
│   ├── gz_bridge.yaml                  # Gazebo ↔ ROS2 topic bridge
│   ├── joystick.yaml                   # Joystick button mappings
│   ├── mapper_params_online_async.yaml # SLAM Toolbox tuned config
│   ├── nav2_params.yaml                # Nav2 planner/controller params
│   ├── JustBot.rviz                    # RViz config (sim mode)
│   └── R2_Navigation.rviz             # RViz config (nav mode)
│
├── description/                        # Robot model (URDF/Xacro)
│   ├── robot.urdf.xacro
│   ├── robot_base.xacro
│   ├── ros2_control.xacro
│   ├── imu.xacro
│   ├── lidar.xacro
│   └── inertial_macros.xacro
│
├── launch/
│   ├── launch_sim.launch.py            # Main simulation launch
│   ├── navigation_launch.py            # Nav2 autonomous navigation
│   ├── localization_launch.py          # Localization on saved map
│   ├── joystick.launch.py
│   └── rsp.launch.py
│
├── maps/                               # Saved SLAM maps
│   └── *.yaml / *.pgm / *.data / *.posegraph
│
├── src/                                # Custom ROS2 Python nodes
│   ├── frontier_explorer.py            # Frontier-based maze exploration
│   ├── slip_detection.py               # LiDAR wheel slip correction
│   └── path_detector.py               # Available direction logger
│
├── worlds/
│   └── Maze.world                      # Gazebo maze environment
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🧠 Architecture

```
Gazebo Simulation
      │
      ├── /scan  (LiDAR)
      ├── /imu/data
      └── /diff_drive_base_controller/odom
               │
               ▼
      slip_detection.py  ←── /scan
               │
               ▼ /odom/corrected
      robot_localization (EKF)
               │
               ▼ /odom/filtered  +  odom→base_footprint TF
      SLAM Toolbox  ←── /scan
               │
               ▼ /map
      Nav2 Stack (MPPI Controller)
               │
               ▼ /cmd_vel
      frontier_explorer.py ──► NavigateToPose action
```

---

## ⚙️ Configuration

### EKF (`config/ekf.yaml`)
Fuses wheel odometry velocities with IMU angular velocity for a smooth, drift-corrected pose estimate. TF publishing is handled by the EKF — `enable_odom_tf: false` is set in the diff_drive controller to avoid conflicts.

### Slip Detection (`src/slip_detection.py`)
Compares consecutive LiDAR scans to detect when the robot is stuck against a wall but the wheels are still reporting motion. When slip is detected, the corrected odometry is published with zeroed velocity and the last known good pose.

| Parameter | Default | Description |
|---|---|---|
| `scan_diff_threshold` | 0.01m | Environment change threshold |
| `velocity_threshold` | 0.005 m/s | Minimum velocity to consider "moving" |
| `stuck_count_threshold` | 5 | Consecutive stuck frames before triggering |
| `odom_cache_size` | 10 | Frames to cache for pose rewind |

### Frontier Explorer (`src/frontier_explorer.py`)
Scores frontiers using `size / distance` — larger unexplored regions that are closer score higher. Goals are validated against the occupancy grid before being sent to Nav2.

| Parameter | Default | Description |
|---|---|---|
| `min_frontier_size` | 5 cells | Filters noise frontiers |
| `min_goal_distance` | 0.5m | Skips already-nearby frontiers |
| `nav_timeout_sec` | 30.0s | Goal timeout before retry |
| `unknown_threshold` | 0.15 | Fraction of unknown space to declare done |

### SLAM Toolbox (`config/mapper_params_online_async.yaml`)
Tuned for tight indoor maze environments — higher map update rate, finer resolution, and more aggressive loop closure detection.

---

## 📝 Notes

- The robot uses a **differential drive** configuration managed via `ros2_control`
- Gazebo topics are bridged to ROS2 using the `gz_bridge` config
- The EKF publishes the `odom → base_footprint` TF — the diff_drive controller has `enable_odom_tf: false`
- Nav2 consumes `/odom/filtered` instead of raw `/odom`

---

## 📄 License

This project is open source. Feel free to use, modify, and build upon it.
