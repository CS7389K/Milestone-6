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

Visual servoing that tracks a bottle by keeping it centered in the camera frame.

```sh
ros2 launch milestone6 part1.launch.py
```

### Part 2: Autonomous Grab & Transport

Complete autonomous sequence to grab and transport an object.

```sh
ros2 launch milestone6 part2.launch.py
```

### Part 3: Voice-Guided Search

Voice-controlled robot search with speech-to-text and text-to-speech.

```sh
# On Remote PC:
ros2 launch milestone6 part3.jetson.launch.py

# On TurtleBot:
ros2 launch milestone6 part3.tb3.launch.py
```
**Note:** Remote PC and TurtleBot need to be connected to the same wifi network.

### Part 4: LLM-Guided Navigation

Natural language command interface using LLaMA to control the robot.

```sh
# On Remote PC:
ros2 launch milestone6 part4.jetson.launch.py

# On TurtleBot:
ros2 launch milestone6 part4.tb3.launch.py

# Send commands using the interactive CLI:
ros2 run milestone6 cli

# Or publish to /user_command topic:
ros2 topic pub /user_command std_msgs/String "data: 'search for the bottle'"
```

**Example Commands** (use lowercase):
- "scan the room for the bottle" → Robot rotates 360° looking for bottle
- "pick up the bottle" → Robot executes grab sequence
- "go to the bear" → Robot navigates toward bear object
- "we are done" → Mission complete

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

