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

Milestone 6 integrates multiple ROS2 subsystems to create a comprehensive robotics platform. This project combines:

- **Teleoperation** ([Milestone 3](https://github.com/CS7389K/Milestone-3)): Command-line interface for robot control
- **Computer Vision** ([Milestone 4](https://github.com/CS7389K/Milestone-4)): YOLO-based object detection
- **LLM Interaction** ([Milestone 5](https://github.com/CS7389K/Milestone-5)): LLaMA integration for natural language processing

The system is designed to run across multiple devices: a remote PC and a Turtlebot Jetson

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

### Terminal 1: Launch the TurtleBot3 OpenManipulator arm
```sh
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

### Terminal 2: Launch a configuration

```sh
# Launches:
# - TeleopPublisher Node: Centralized teleop service for base and arm control
# - YOLO Publisher Node: Captures camera frames and publishes object detections
# - TeleopBase Node: Subscribes to detections and controls robot base movement
ros2 launch milestone6 part1.launch.py
```

```sh
# Launches:
# - TeleopPublisher Node: Centralized teleop service for base and arm control
# - YOLO Publisher Node: Captures camera frames and publishes object detections
# - TeleopBase Node: Subscribes to detections and controls robot base movement
# - TeleopArm Node: Automatically grabs detected objects
ros2 launch milestone6 part2.launch.py
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

