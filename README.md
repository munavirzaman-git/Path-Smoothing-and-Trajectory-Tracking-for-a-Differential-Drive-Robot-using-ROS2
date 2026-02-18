# 🤖 Path Smoothing & Trajectory Tracking — Differential Drive Robot (ROS2)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Python](https://img.shields.io/badge/Python-3.x-blue?logo=python)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-orange)
![TurtleBot3](https://img.shields.io/badge/Robot-TurtleBot3-green)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

A complete ROS2 implementation of **B-spline path smoothing**, **time-parameterized trajectory generation**, and **Pure Pursuit trajectory tracking** for a differential drive robot in Gazebo simulation.

---

## 📌 Overview

Discrete 2D waypoints are converted into a smooth, continuous path using a **cubic B-spline**, then time-parameterized to form a full trajectory. A **Pure Pursuit controller** tracks the trajectory in real time, publishing velocity commands to the robot.

Performance is evaluated both **visually** (RViz overlay of reference vs. actual path) and **quantitatively** (cross-track error analysis).

---

## 🎯 Features

- ✅ Smooth path generation from discrete waypoints (cubic B-spline)
- ✅ Arc-length based time parameterization
- ✅ Pure Pursuit trajectory tracking controller
- ✅ Real-time RViz visualization (reference vs. actual path)
- ✅ Cross-track error (CTE) computation and analysis
- ✅ Modular ROS2 node architecture

---

## 🧱 System Architecture

```
Waypoints
   │
   ▼
[path_smoother_node] ──► /smoothed_path
   │
   ▼
[trajectory_generator_node] ──► /trajectory
   │
   ▼
[trajectory_tracker_node] ──► /cmd_vel ──► TurtleBot3
   │
   ▼
/actual_path + /lookahead_point (RViz)
```

### ROS2 Nodes

| Node | Function |
|------|----------|
| `path_smoother_node` | Converts waypoints → smooth B-spline path |
| `trajectory_generator_node` | Adds time parameterization to path |
| `trajectory_tracker_node` | Pure Pursuit controller → `/cmd_vel` |

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/smoothed_path` | `nav_msgs/Path` | B-spline smoothed path |
| `/trajectory` | `nav_msgs/Path` | Time-parameterized reference path |
| `/actual_path` | `nav_msgs/Path` | Robot followed path |
| `/lookahead_point` | `visualization_msgs/Marker` | Pure pursuit target point |
| `/odom` | `nav_msgs/Odometry` | Robot pose |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command |

---

## ⚙️ Setup & Installation

### Prerequisites

- ROS2 Humble
- TurtleBot3 packages
- Gazebo
- Python3 with `scipy` and `numpy`

```bash
sudo apt install ros-humble-turtlebot3* python3-scipy
export TURTLEBOT3_MODEL=waffle
```

### Build

```bash
cd ~/origin_ws
colcon build
source install/setup.bash
```

---

## 🚀 Running the System

**1. Launch Gazebo simulation:**
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**2. Launch the trajectory system:**
```bash
ros2 launch trajectory_control_pkg bringup.launch.py
```

**3. Launch RViz:**
```bash
rviz2
```

RViz configuration:
- **Fixed Frame** → `odom`
- **Path** → `/trajectory` (Green — reference path)
- **Path** → `/actual_path` (Red — actual robot path)
- **Marker** → `/lookahead_point` (Yellow)
- **Odometry** → `/odom`

---

## 🧠 Algorithm Details

### Path Smoothing — B-Spline

Discrete waypoints are interpolated into a smooth parametric curve using **cubic B-spline** interpolation.

**Why B-spline?**
- Guarantees C² continuity
- Produces curvature-bounded paths
- Eliminates sharp turns
- Widely used in robotic motion planning

### Trajectory Generation

Arc-length based time parameterization:

$$t_i = \frac{\sum d_i}{v}$$

Where $d_i$ = distance between consecutive points, $v$ = constant velocity (0.25 m/s)

### Trajectory Tracking — Pure Pursuit Controller

A lookahead point at distance $L_d$ is selected and curvature is computed:

$$\kappa = \frac{2 y_r}{L_d^2}, \quad \omega = v \cdot \kappa$$

**Controller Parameters:**

| Parameter | Value |
|-----------|-------|
| Lookahead distance | 0.4 m |
| Linear velocity | 0.25 m/s |
| Max angular velocity | ±1.5 rad/s |

**Why Pure Pursuit?**
- Simple and stable for differential drive robots
- Handles curved paths effectively
- Computationally efficient for real-time control

---

## 📊 Performance Results

### Cross-Track Error (CTE)

| Test Case | Max CTE | Mean CTE |
|-----------|---------|----------|
| Straight Line | < 0.05 m | Low |
| S-Curve | < 0.15 m | Moderate |

### Test Cases

**Test 1 — Straight Line**
- Waypoints: `(0,0) → (4,0)`
- Result: Stable tracking with minimal oscillation

**Test 2 — S-Curve**
- Curved waypoint set
- Result: Smooth tracking with small error and no oscillations

---

## 📷 Results

<img width="1920" height="1080" alt="S-Curve" src="https://github.com/user-attachments/assets/87822747-f12c-4b32-a0c7-70e464eb1223" />

- RViz plots
---

## ⚠️ Error Handling

- Stop condition near goal
- Lookahead point validation
- Angular velocity saturation
- Empty trajectory protection

---

## 🔧 Extension to Real Robot

To deploy on a physical TurtleBot3:

- Use real `/odom` and `/scan` topics
- Tune lookahead distance based on actual speed
- Implement trapezoidal velocity profile for smooth acceleration
- Add wheel slip compensation
- Integrate with Nav2 lifecycle nodes for robustness

---

## ⭐ Obstacle Avoidance (Extra Credit)

Proposed approach:
- Use LiDAR `/scan` to detect obstacles within the lookahead region
- Shift the lookahead point laterally away from obstacles
- Alternatively, integrate **Dynamic Window Approach (DWA)** as a local planner while using the trajectory as a global reference path


---

## 👤 Author

**Munavir Zaman**  
ROS2 · Robotics · Autonomous Navigation

---

## 📄 License

This project is licensed under the MIT License.
