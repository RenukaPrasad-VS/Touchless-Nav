# Touchless Navigation (ROS 2)

A ROS 2 project for gesture control and touchless goal selection with a TurtleBot and Nav2. This repository is maintained by [@RenukaPrasad-VS](https://github.com/RenukaPrasad-VS).

## Upstream Reference

This project is based on [GGANS-Touchless-Navigation by God-Official](https://github.com/God-Official/GGANS-Touchless-Navigation). That repository is credited as the upstream reference; this repository contains the changes and maintenance described below.

## Changes in This Repository

- Updated the simulation launch setup for Ignition Gazebo / Gazebo Fortress and configured the resource path for the packaged world meshes.
- Stabilized gesture teleoperation with a confidence threshold, multi-frame gesture confirmation, monotonic timestamps, and a stop when detections become uncertain or stale.
- Improved gesture recognition across different hand sizes with configurable MediaPipe detection/tracking thresholds and confidence-weighted temporal voting.
- Reworked the touchless poke detector to smooth relative finger depth and require retraction before another click.
- Changed clicked navigation goals to use Nav2's `NavigateToPose` action, including goal acceptance and result reporting.
- Fixed package installation issues needed to build and run the workspace.

## Tech Stack
- ROS 2 Humble Hawksbill
- Ignition Gazebo Fortress
- Nav2 (Navigation2)
- OpenCV
- Mediapipe
- TensorFlow Lite

## System Flow
```
Camera → Gesture Recognition → Navigation Goal / Command
        ↓
     Nav2 Planner → Controller → /cmd_vel → Robot
```

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble installed and sourced

## Installation
### Install ROS & Simulation Dependencies
```
    sudo apt update
    sudo apt install -y \
        ros-humble-nav2* \
        ros-humble-ros-ign \
        ros-humble-ros-ign-bridge \
        ros-humble-ros-ign-gazebo \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        python3-opencv
```
### Install Vision Libraries
```
pip3 install mediapipe tensorflow tensorflow-lite
```

## Workspace Setup
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/RenukaPrasad-VS/Touchless-Nav.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the Project
### Launch Gazebo Simulation
```
ros2 launch turtlebot_description gazebo.launch.py
```

### Launch Navigation Stack
```
ros2 launch turtlebot_nav2 nav2.launch.py
```

### Launch Camera Node
```
ros2 launch camera_pub camera_pub.launch.py
```

### Launch Gesture Control
```
ros2 launch gesture_control gesture_control.launch.py
```

Gesture recognition can be tuned at launch. The detector thresholds default to `0.45` for hand detection and presence, which helps detect hands that appear smaller in the camera frame. Gesture labels still require a confidence of `0.65` and confirmation across 3 frames. For example:
```
ros2 launch gesture_control gesture_control.launch.py min_hand_detection_confidence:=0.4 min_hand_presence_confidence:=0.4 stable_frames:=4
```

### Launch GGANS (Touchless Navigation)
```
ros2 launch touchless_nav touchless_nav.launch.py
```

## Features
- Vision-based, touchless robot control
- Autonomous navigation with obstacle avoidance
- Gesture-driven goal selection
- Fully simulated and reproducible
- Modular ROS 2 design

## Maintainer

[@RenukaPrasad-VS](https://github.com/RenukaPrasad-VS)
