# VLA Object Navigation

## Start with the original repository

Clone the original GGANS Touchless Navigation repository from
[GitHub](https://github.com/God-Official/GGANS-Touchless-Navigation):

```bash
git clone https://github.com/God-Official/GGANS-Touchless-Navigation.git
cd GGANS-Touchless-Navigation
```

## VLA work

This README describes the VLA object-navigation application added to the
original project. It uses a vision-language model to identify a named object
and Nav2 to navigate toward it.

This ROS 2 package replaces cardinal-direction commands with object-target
navigation. Send a text/voice transcript such as `go to the chair`. A local
vision-language model finds that object in the current RGB image and returns a
bounding box. The node samples aligned depth near the lower part of the box,
projects the pixel into 3D using `CameraInfo`, transforms it into `map`, and
sends a standoff goal to Nav2. Nav2 remains responsible for path planning and
obstacle avoidance.

This is a practical VLA-style ROS integration for simulation. The model
grounds language in an image, while calibrated depth and TF provide a metric
target pose. It does not require a robot-specific training dataset. It is not a
direct motor-action policy; Nav2 executes the goal.

## Install the local vision model

Install [Ollama](https://ollama.com/download), then pull a local vision model:

```bash
ollama pull qwen3-vl:4b
ollama serve
```

Use Ollama 0.12.7 or newer for the `qwen3-vl` model family. The `4b` tag is
about 3.3 GB on the Ollama library, so it is a reasonable desktop starting
point; the model runs locally and accepts both text and images.

The ROS node calls Ollama's local `/api/chat` endpoint. No image or command is
sent to a cloud service. Ensure Python Pillow is available (`sudo apt install
python3-pil`). An NVIDIA GPU is optional for running the code, but recommended
for responsive inference.

## Build

From the colcon workspace root (the directory containing `build/`, `install/`,
and `log/`):

```bash
rosdep install --from-paths GGANS-Touchless-Navigation/vla_nav --ignore-src -r -y
colcon build --packages-up-to vla_nav
source install/setup.bash
```

## Run in Gazebo

Stop any currently running Gazebo session first; a running world cannot be
switched in place. The VLA demo launch starts Gazebo with a copy of the
existing world plus three target objects (chair, red box, and table). The
original world file is unchanged. Start this instead of
`turtlebot_description/gazebo.launch.py`:

```bash
ros2 launch vla_nav vla_sim.launch.py
```

In another terminal, start Nav2:

```bash
ros2 launch turtlebot_nav2 nav2.launch.py use_sim_time:=true
```

In a separate terminal, start Ollama (the model should already be pulled):

```bash
ollama serve
```

In another terminal, start the VLA node:

```bash
ros2 launch vla_nav vla_nav.launch.py
```

The VLA node settings are loaded from
`vla_nav/config/vla_nav.yaml`; the launch command needs no parameter
overrides. Edit that file to change topics, TF frames, model, simulation time,
or approach distance. For a real robot, set `use_sim_time` to `false`, update
the camera topics and `base_frame`, then rebuild and source the workspace.

The target is selected by naming it in your command; try one of these:

```bash
ros2 topic pub --once /voice/recognized_text std_msgs/msg/String "{data: 'go to the chair'}"
ros2 topic pub --once /voice/recognized_text std_msgs/msg/String "{data: 'go to the red box'}"
ros2 topic pub --once /voice/recognized_text std_msgs/msg/String "{data: 'go to the table'}"
```

The objects are placed in the robot's forward camera view. Restart the VLA demo
launch to reset the world and targets.

Cancel a goal at any time:

```bash
ros2 topic pub --once /voice/recognized_text std_msgs/msg/String "{data: 'stop'}"
```

The default camera topics match this repository's Gazebo bridge:

| Parameter | Default |
| --- | --- |
| `image_topic` | `/camera/image` |
| `depth_topic` | `/camera/depth_image` |
| `camera_info_topic` | `/camera/camera_info` |
| camera frame | `CameraInfo.header.frame_id` |
| map frame | `map` |
| robot base frame | `base_footprint` |

RGB and depth images must be aligned and have the same dimensions. Supported
depth encodings are `32FC1` (meters) and `16UC1` (millimeters). The camera
frame must have a valid TF path to `map`. `standoff_m` is set to `0.15` m and controls how far Nav2
stops short of the detected object on the side visible to the robot and faces
toward the object. The depth sample is taken toward the lower part of its image
box to better estimate a ground level approach. Objects without valid depth are
rejected.

The demo config sets `camera_frame_uses_optical_convention: false` to account
for the zero-rotation camera joint in this repository's Gazebo URDF. For a real
camera whose TF follows the standard optical axes, set it to `true`.

## Connect an actual robot later

Keep the Nav2 action interface. Change only the camera topics, frame names,
map, and simulation clock setting to match the robot. For example, a robot
publishing aligned RealSense topics could be launched like this (replace the
topic and frame names with the values from that robot):

Edit `vla_nav/config/vla_nav.yaml` for the real robot, then run:

```bash
ros2 launch vla_nav vla_nav.launch.py
```

The actual robot must also run Nav2, publish calibrated RGB-D data, and provide
TF from its camera optical frame through its base to the map. Make sure Nav2's
costmaps use the robot's real lidar/depth sensors. The VLM's bounding box is a
target estimate, not a safety sensor.

An application can publish voice-recognition results with this minimal ROS 2
Python snippet:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VLACommandInput(Node):
    def __init__(self):
        super().__init__('vla_command_input')
        self.publisher = self.create_publisher(String, '/voice/recognized_text', 10)

    def send(self, transcript):
        msg = String()
        msg.data = transcript  # e.g. "go to the chair"
        self.publisher.publish(msg)

rclpy.init()
node = VLACommandInput()
node.send('go to the chair')
node.destroy_node()
rclpy.shutdown()
```

Check actual robot topic names with `ros2 topic list`, camera calibration with
`ros2 topic echo /camera/color/camera_info --once`, and TF with
`ros2 run tf2_ros tf2_echo map camera_color_optical_frame`.

## Author

Renuka Prasad VS

Robotics and ROS 2 Developer | Computer Vision | Manipulation | Autonomous Navigation

## Reference

Original project: [GGANS Touchless Navigation](https://github.com/God-Official/GGANS-Touchless-Navigation)
