# Milestone 6

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue.svg)](https://docs.ros.org/en/foxy/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> An integrated ROS2 robotics system combining teleoperation, computer vision (YOLO), and AI-powered natural language interaction (LLaMA) for Turtlebot platforms.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building](#building)
- [Usage](#usage)
- [Resources](#resources)
- [License](#license)

## Overview

Milestone 6 integrates multiple ROS2 subsystems to create a comprehensive robotics platform with four progressive parts:

1. **Part 1: Visual Servoing** - Bottle tracking using camera-based feedback
2. **Part 2: Autonomous Grab & Transport** - Complete object manipulation mission
3. **Part 3: Voice-Guided Search** - Speech interaction with Whisper and Espeak
4. **Part 4: LLM-Guided Navigation** - Natural language commands via LLaMA

This project combines:

- **Teleoperation** ([Milestone 3](https://github.com/CS7389K/Milestone-3)): Command-line interface for robot control
- **Computer Vision** ([Milestone 4](https://github.com/CS7389K/Milestone-4)): YOLO-based object detection
- **LLM Interaction** ([Milestone 5](https://github.com/CS7389K/Milestone-5)): LLaMA integration for natural language processing using [Llama-2-7B-32K-Instruct-GGUF](https://huggingface.co/TheBloke/Llama-2-7B-32K-Instruct-GGUF)

The system is designed to run across multiple devices: a remote PC and a Turtlebot with Jetson

## Prerequisites

- **Operating System**: Ubuntu 20.04 LTS
- **ROS2**: Foxy
- **Python**: 3.8
- **Hardware**: 
  - Remote PC
  - Turtlebot with a Jetson
  - Secondary Jetson (optional)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/CS7389K/Milestone-6.git
cd Milestone-6
```

### 2. Set Up Ubuntu 20.04

ROS2 Foxy requires Ubuntu 20.04. If you're using Windows, set up WSL2:

```bash
wsl --install -d Ubuntu-20.04
```

**Optional**: Move WSL to a different drive (e.g., F: drive):

```bash
wsl --manage Ubuntu-20.04 --move F:\WSL
```

### 3. Install ROS2 Foxy

Run the installation script provided in the repository:

```bash
sh install-ros2-foxy-desktop.sh
```

Alternatively, follow the [official ROS2 Foxy installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

## Building

Navigate to the project root directory and build the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

After building the project, source the workspace in each terminal session:

```bash
source install/setup.sh
```

### Part 1: Visual Servoing for Bottle Tracking

**Objective**: Implement visual servoing that continuously tracks a bottle by rotating the robot to keep it centered in the camera frame.

```sh
# On TurtleBot (includes hardware bringup by default):
ros2 launch milestone6 part1.launch.py

# Without hardware bringup:
ros2 launch milestone6 part1.launch.py include_hardware:=false
```

**What it launches**:
- Hardware bringup (unless include_hardware:=false)
- TeleopPublisher Node: Centralized teleop service for base control
- YOLO Publisher Node: Captures camera frames and publishes bottle detections
- Part1 Node: Visual servoing controller (centering only, no approach)

### Part 2: Complete Object Grab and Transport Mission

**Objective**: Complete autonomous grab and transport sequence with mission states: IDLE → CENTERING → APPROACHING → GRABBING → TRANSPORTING → RELEASING → DONE

```sh
# On TurtleBot (includes hardware bringup by default):
ros2 launch milestone6 part2.launch.py

# Without hardware bringup:
ros2 launch milestone6 part2.launch.py include_hardware:=false
```

**What it launches**:
- Hardware bringup (unless include_hardware:=false)
- YOLO Publisher Node: Object detection using camera
- Part 2 Mission Node: Complete autonomous grab and transport sequence

### Part 3: Voice-Guided Object Search

**Objective**: Voice-controlled robot search with speech-to-text (Whisper) and text-to-speech (Espeak) capabilities.

```sh
# On Remote PC (Whisper + Espeak):
ros2 launch milestone6 part3.jetson.launch.py

# On TurtleBot (Hardware + YOLO + Mission):
ros2 launch milestone6 part3.tb3.launch.py
```

**What it launches**:

*Remote PC (part3.jetson.launch.py)*:
- Whisper Publisher: Speech-to-text transcription service
- Espeak Subscriber: Text-to-speech output service

*TurtleBot (part3.tb3.launch.py)*:
- Hardware bringup (unless include_hardware:=false)
- YOLO Publisher: Object detection
- Part 3 Mission Node: Voice-guided search with audio recording

### Part 4: LLM-Guided Search and Retrieval

**Objective**: Natural language command interface using LLaMA to translate user commands into atomic robot actions.

```sh
# On Remote PC (LLaMA + Espeak):
ros2 launch milestone6 part4.jetson.launch.py

# On TurtleBot (Hardware + YOLO + Mission):
ros2 launch milestone6 part4.tb3.launch.py

# Send commands using the interactive CLI (recommended):
python3 src/milestone6/milestone6/nlp/cli.py

# Or send individual commands from another terminal using topics:
ros2 topic pub /user_command std_msgs/String "data: 'Scan for the bottle'"
ros2 topic pub /user_command std_msgs/String "data: 'Turn left'"
ros2 topic pub /user_command std_msgs/String "data: 'Move forward'"
ros2 topic pub /user_command std_msgs/String "data: 'Pick up the bottle'"
ros2 topic pub /user_command std_msgs/String "data: 'Go to the bear which is 2 meters ahead'"
ros2 topic pub /user_command std_msgs/String "data: 'Search for the bear'"
ros2 topic pub /user_command std_msgs/String "data: 'Place the bottle'"
ros2 topic pub /user_command std_msgs/String "data: 'We are done'"
```

**What it launches**:

*Remote PC (part4.jetson.launch.py)*:
- LLaMA Publisher: Translates user text to atomic robot actions
- Espeak Subscriber: Text-to-speech output service

*TurtleBot (part4.tb3.launch.py)*:
- Hardware bringup (unless include_hardware:=false)
- YOLO Publisher: Multi-object detection (bottle, bear, mouse)
- Part 4 Mission Node: LLM-guided search and retrieval

**Sample Natural Language Prompts**:

The LLaMA model translates natural language into atomic robot actions. Here are example prompts you can use:

*Exploration & Search:*
- "Scan the room for the bottle"
- "Look around and find the bear"
- "Turn 360 degrees to search"
- "Rotate left slowly"
- "Turn right and look for objects"

*Movement:*
- "Move forward"
- "Go straight for a bit"
- "Back up slowly"
- "Stop moving"

*Object Manipulation:*
- "Pick up the bottle"
- "Grab the object in front of you"
- "Place the bottle down"
- "Release what you're holding"

*Navigation:*
- "Go to the bear"
- "Navigate to the mouse which is 2 meters ahead"
- "Move toward the bear on your left"

*Combined Tasks:*
- "Find the bottle, pick it up, and bring it to the bear"
- "Search for the mouse and grab it"
- "Go to the bear and place the bottle there"

*Mission Control:*
- "We are done"
- "Mission complete"
- "Stop and finish"

## Common Launch Parameters

All launch files support various parameters to customize behavior. Here are some common ones:

### YOLO Detection Parameters
- `yolo_model:=<path>` - Path to YOLO model file (default: yolo11n.pt)
- `image_width:=<int>` - Camera image width in pixels (default: 1280)
- `image_height:=<int>` - Camera image height in pixels (default: 720)
- `display:=<bool>` - Display YOLO detection window (default: true)
- `tracking_classes:=<str>` - Comma-separated COCO class IDs (default varies by part)

### Camera Parameters
- `camera_backend:=<str>` - Camera backend: gstreamer or opencv (default: gstreamer)
- `camera_device:=<int>` - Camera device ID (default: 1)
- `gstreamer_pipeline:=<str>` - Custom GStreamer pipeline (default: '')

### Motion Parameters
- `speed:=<float>` - Linear movement speed in m/s (default: 0.05)
- `turn_speed:=<float>` - Angular velocity for turning rad/s (default: 0.25)
- `center_tolerance:=<int>` - Centering tolerance in pixels (default: 30)
- `target_bbox_width:=<int>` - Target bounding box width in pixels (default: 365)

### Hardware
- `include_hardware:=<bool>` - Include hardware.launch.py (default: true)

Example usage:
```sh
ros2 launch milestone6 part1.launch.py yolo_model:=yolo11s.pt display:=false speed:=0.1
```

## Resources

### ROS2 Foxy Documentation

- [Installation Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Building Packages with Colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS2 Actions](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html)

### Python & Development

- [Python 3.8 Documentation](https://docs.python.org/3.8/)
- [ROS2 Python Client Library](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## Troubleshooting

### Launch File Not Found Error

**Problem**: When running `ros2 launch milestone6 <launch_file>.launch.py`, you get an error like:
```
File <launch_file>.launch.py was not found in the share directory of the package
```

**Cause**: This can occur when an old installation of the package exists in a different location (e.g., `milestone6` vs `Milestone-6` directory name mismatch).

**Solution**:
1. Find the old installation location:
   ```bash
   ros2 pkg prefix milestone6
   ```

2. Remove the old installation:
   ```bash
   rm -rf <old_installation_path>
   ```

3. Rebuild the package:
   ```bash
   colcon build --packages-select milestone6
   source install/setup.bash
   ```

4. Verify the new installation location:
   ```bash
   ros2 pkg prefix milestone6
   ```

5. Confirm launch files are installed:
   ```bash
   ls -la $(ros2 pkg prefix milestone6)/share/milestone6/launch/
   ```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Built with ❤️ using ROS2 Foxy and Python 3.8**

