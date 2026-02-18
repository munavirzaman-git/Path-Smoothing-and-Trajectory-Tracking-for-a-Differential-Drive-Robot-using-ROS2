# Path Smoothing and Trajectory Tracking for a Differential Drive Robot (ROS2)

## 📌 Overview
This project implements path smoothing and trajectory tracking for a differential drive robot using **ROS2 Humble** and **TurtleBot3** in Gazebo simulation.

A set of discrete 2D waypoints is converted into a smooth continuous path using a **B-spline**, followed by **time parameterization** to generate a trajectory.  
A **Pure Pursuit controller** is used to track the trajectory and publish velocity commands to the robot.

Both **visual** and **quantitative** evaluations are performed using RViz and cross-track error (CTE) analysis.

---

## 🎯 Objectives

- Generate a smooth path from discrete waypoints  
- Create a time-parameterized trajectory  
- Implement a trajectory tracking controller  
- Validate performance in simulation  
- Visualize reference vs actual path in RViz  

---

## 🧱 System Architecture

### ROS2 Nodes

| Node | Function |
|------|----------|
`path_smoother_node` | Converts waypoints → smooth B-spline path |
`trajectory_generator_node` | Adds time parameterization to path |
`trajectory_tracker_node` | Pure pursuit controller → `/cmd_vel` |

---

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
`/smoothed_path` | `nav_msgs/Path` | B-spline smoothed path |
`/trajectory` | `nav_msgs/Path` | Time-parameterized reference path |
`/actual_path` | `nav_msgs/Path` | Robot followed path |
`/lookahead_point` | `visualization_msgs/Marker` | Pure pursuit target point |
`/odom` | `nav_msgs/Odometry` | Robot pose |
`/cmd_vel` | `geometry_msgs/Twist` | Velocity command |

---

## 🧠 Algorithms Used

### 1️⃣ Path Smoothing – B-Spline
Discrete waypoints are converted into a smooth parametric curve using cubic B-spline interpolation.

**Advantages:**
- C² continuity  
- Curvature-bounded path  
- Eliminates sharp turns  

The smoothed path is sampled uniformly to generate a continuous trajectory.

---

### 2️⃣ Trajectory Generation
Arc-length based time parameterization:

\[
t_i = \frac{\sum d_i}{v}
\]

Where:
- \( d_i \) = distance between consecutive points  
- \( v \) = constant velocity (0.25 m/s)

This produces a time-stamped trajectory suitable for tracking.

---

### 3️⃣ Trajectory Tracking – Pure Pursuit
A lookahead point at distance \( L_d \) is selected and curvature is computed:

\[
\kappa = \frac{2y_r}{L_d^2}
\]

\[
\omega = v \cdot \kappa
\]

**Controller Parameters**

| Parameter | Value |
|-----------|--------|
Lookahead distance | 0.4 m |
Linear velocity | 0.25 m/s |
Max angular velocity | ±1.5 rad/s |

---

## 👀 Visualization (RViz)

- **Green Path** → Reference trajectory  
- **Red Path** → Actual robot path  
- **Yellow Marker** → Lookahead point  

The close overlap between the red and green paths indicates low tracking error.

---

## 📊 Performance Evaluation

### Cross-Track Error (CTE)

CTE is computed as the minimum distance between the robot pose and the reference trajectory at each timestep.

| Test Case | Max CTE | Mean CTE |
|-----------|---------|----------|
Straight Line | < 0.05 m | Low |
S-Curve | < 0.15 m | Moderate |

*(Values based on simulation results)*

---

## 🧪 Test Cases

### Test 1 – Straight Line
Waypoints: `(0,0) → (4,0)`  
Result: Stable tracking with minimal oscillation.

### Test 2 – S-Curve
Curved waypoint set  
Result: Controller follows curvature smoothly with small tracking error.

---

## ⚠️ Error Handling

- Stop condition near goal  
- Lookahead validation  
- Angular velocity saturation  
- Empty trajectory protection  

---

## 🤖 Real Robot Extension

To deploy on a real TurtleBot3:

- Use real `/odom` and `/scan` topics  
- Tune lookahead based on speed  
- Implement trapezoidal velocity profile  
- Add wheel slip compensation  
- Integrate with Nav2 lifecycle nodes  

---

## ⭐ Obstacle Avoidance (Proposed Extension)

- Use LiDAR `/scan` to detect obstacles in the lookahead region  
- Shift lookahead point laterally away from obstacles  
- Alternatively, integrate **DWA** as a local planner while using the trajectory as reference  

---

## 🧰 AI Tools Used

AI tools were used to:

- Compare path smoothing methods  
- Design ROS2 node architecture  
- Derive Pure Pursuit equations  
- Tune controller parameters  

All code was implemented, tested, and validated manually in simulation.

---

## ▶️ How to Run

### 1️⃣ Build Workspace

```bash
cd ~/origin_ws
colcon build
source install/setup.bash

