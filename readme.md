# Gesture-Guided Autonomous Navigation System (GGANS)

GGANS is a ROS 2 project for touchless navigation of a TurtleBot3 using a camera, hand gestures, and Nav2. It uses the installed TurtleBot3 Gazebo and Navigation2 packages for the robot, simulation, and navigation stack.

## Requirements

- Ubuntu 22.04 and ROS 2 Humble
- TurtleBot3 Gazebo, Navigation2, and teleop packages
- A webcam
- Python MediaPipe for hand tracking

## Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-turtlebot3-teleop \
  ros-humble-cv-bridge \
  python3-opencv
```

Install MediaPipe:

```bash
python3 -m pip install mediapipe
```

ROS `cv_bridge` must use the system OpenCV installation. If you installed pip versions of OpenCV and `cv_bridge` reports an encoding error, remove those conflicting wheels:

```bash
python3 -m pip uninstall opencv-python opencv-contrib-python
python3 -c "import cv2; print(cv2.__version__, cv2.__file__)"
```

The OpenCV path should be under `/usr/lib/python3/dist-packages`, not `~/.local`.

## Build the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone your own GitHub repository (replace these placeholders with its URL)
git clone https://github.com/YOUR_USERNAME/YOUR_REPOSITORY.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

In each terminal used below, source the workspace and set the robot model:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

## Run the simulation and navigation

Start Gazebo in one terminal:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Start Nav2 in a second terminal:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

In RViz, initialize localization before sending a goal. Select **2D Pose Estimate**, click the robot's current location on the map, drag in the direction it faces, and release. Wait for AMCL to settle.

## Use touchless navigation

Start the camera publisher in a third terminal:

```bash
ros2 launch camera_pub camera_pub.launch.py
```

Start the touchless navigation node in a fourth terminal:

```bash
ros2 launch touchless_nav touchless_nav.launch.py
```

The touchless map window displays the map over the camera image. To send a goal, point at a target with your index finger, briefly poke the fingertip toward the camera, then pull it back. Each air click sends one goal to Nav2. Moving your hand without the poke should not send a goal.

## Optional direct teleoperation

For direct gesture teleoperation, start the camera and then run:

```bash
ros2 launch gesture_control gesture_control.launch.py
```

The gesture teleop node publishes directly to `/cmd_vel`. Stop it while using Nav2 goal navigation, since both can send commands to the robot. Keyboard teleop is also available:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Use keyboard or gesture teleoperation instead of Nav2 at a time.

## Features

- Hand gesture recognition with MediaPipe
- Air-click map goals for Nav2
- Nav2 path planning and obstacle avoidance
- TurtleBot3 Gazebo simulation

## Author

Renuka Prasad V S
