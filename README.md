# ROS 2 Color-Based Object Search & Navigation
### TurtleBot3 Burger | ROS 2 Humble | Gazebo | Nav2

---

## Overview

The robot goes through 3 states automatically:

```
SEARCHING  →  (red ball detected)  →  DETECTED  →  NAVIGATING  →  DONE
```

| State | Behaviour |
|---|---|
| **SEARCHING** | Rotates in place at 0.4 rad/s, analysing camera frames |
| **DETECTED** | Stops immediately, triggers Nav2 goal |
| **NAVIGATING** | Nav2 drives robot to predefined pose near ball |
| **DONE** | Task complete, node idle |

---

## Prerequisites — Install these first

```bash
# 1. TurtleBot3 packages
sudo apt install ros-humble-turtlebot3 \
                 ros-humble-turtlebot3-simulations \
                 ros-humble-turtlebot3-gazebo

# 2. Nav2
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup

# 3. Python deps
sudo apt install python3-opencv ros-humble-cv-bridge

# 4. Set robot model (add to ~/.bashrc too)
export TURTLEBOT3_MODEL=burger
```

---

## Build the Package

```bash
# 1. Clone / copy this package into your workspace
mkdir -p ~/ros2_ws/src
cp -r red_ball_nav ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select red_ball_nav

# 3. Source
source install/setup.bash
```

---

## Spawn the Red Ball in Gazebo

After Gazebo is running:

```bash
ros2 run gazebo_ros spawn_entity.py \
    -entity red_ball \
    -file ~/ros2_ws/src/red_ball_nav/red_ball_nav/red_ball.sdf \
    -x 1.5 -y 0.0 -z 0.1
```

---

## Option A — Run Everything with One Launch File

```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash

ros2 launch red_ball_nav red_ball_nav.launch.py
```

---

## Option B — Run Step by Step (Recommended for Debugging)

Open **4 separate terminals**, source ROS 2 in each:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

**Terminal 1 — Gazebo:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 — Spawn red ball:**
```bash
ros2 run gazebo_ros spawn_entity.py \
    -entity red_ball \
    -file ~/ros2_ws/src/red_ball_nav/red_ball_nav/red_ball.sdf \
    -x 1.5 -y 0.0 -z 0.1
```

**Terminal 3 — Nav2:**
```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=true \
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
```
> In RViz2, use **2D Pose Estimate** to set the initial robot pose on the map before proceeding.

**Terminal 4 — Our Node:**
```bash
ros2 launch red_ball_nav node_only.launch.py
```

---

## Tuning Parameters

Edit `node_only.launch.py` or pass on the CLI:

| Parameter | Default | Description |
|---|---|---|
| `rotate_speed` | 0.4 | Angular velocity (rad/s) while searching |
| `goal_x` | 1.5 | Nav2 goal X coordinate (meters) |
| `goal_y` | 0.0 | Nav2 goal Y coordinate (meters) |
| `goal_yaw` | 0.0 | Goal heading (radians) |
| `min_contour_area` | 500 | Min red blob size in pixels² to count as detection |

---

## Verify Topics

```bash
# Confirm camera is publishing
ros2 topic echo /camera/image_raw --once

# Monitor cmd_vel (should see angular.z = 0.4 while searching)
ros2 topic echo /cmd_vel

# Monitor node state in logs
ros2 node info /color_search_node
```

---

## HSV Tuning for Red Detection

If detection is unreliable, tune the HSV thresholds in `color_search_node.py`:

```python
RED_LOWER1 = np.array([0,   120,  70])   # Hue 0–10
RED_UPPER1 = np.array([10,  255, 255])
RED_LOWER2 = np.array([170, 120,  70])   # Hue 170–180
RED_UPPER2 = np.array([180, 255, 255])
```

To interactively find the right values, run this helper in a new terminal
while the simulation is running:

```python
# hsv_tuner.py  (run standalone: python3 hsv_tuner.py)
import cv2, numpy as np

def nothing(x): pass
cap = cv2.VideoCapture(0)  # replace with your image source
cv2.namedWindow("Trackbars")
for name in ["LH","LS","LV","UH","US","UV"]:
    cv2.createTrackbar(name, "Trackbars", 0, 255, nothing)
cv2.setTrackbarPos("UH","Trackbars",179)
cv2.setTrackbarPos("US","Trackbars",255)
cv2.setTrackbarPos("UV","Trackbars",255)
while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l = np.array([cv2.getTrackbarPos(n,"Trackbars") for n in ["LH","LS","LV"]])
    u = np.array([cv2.getTrackbarPos(n,"Trackbars") for n in ["UH","US","UV"]])
    mask = cv2.inRange(hsv, l, u)
    cv2.imshow("mask", mask); cv2.imshow("frame", frame)
    if cv2.waitKey(1) == 27: break
```

---

## File Structure

```
red_ball_nav/
├── red_ball_nav/
│   ├── __init__.py
│   ├── color_search_node.py   ← Main node (search + detect + navigate)
│   └── red_ball.sdf           ← Gazebo red ball model
├── launch/
│   ├── red_ball_nav.launch.py ← Full launch (Gazebo + Nav2 + node)
│   └── node_only.launch.py    ← Node only (for step-by-step testing)
├── resource/
│   └── red_ball_nav
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Common Issues & Fixes

| Problem | Fix |
|---|---|
| `package 'turtlebot3_gazebo' not found` | `sudo apt install ros-humble-turtlebot3-simulations` |
| Camera topic not found | Check with `ros2 topic list` — may be `/camera/rgb/image_raw` |
| Nav2 rejects goal | Set initial pose in RViz2 with **2D Pose Estimate** first |
| Robot doesn't stop | Increase `min_contour_area` — false positives in environment |
| Ball not visible | Position ball within camera FOV; reduce `min_contour_area` |
