# Milestone 6

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-blue.svg)](https://docs.ros.org/en/foxy/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> An integrated ROS2 robotics system combining teleoperation, computer vision (YOLO), and AI-powered natural language interaction (LLaMA) for Turtlebot platforms.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Running](#running)
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

## Running

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
source install/setup.bash
```

### Running the Server

```bash
ros2 run milestone6 server
```

### Running the Client

```bash
ros2 run milestone6 client
```
## Resources

### ROS2 Foxy Documentation

- [Installation Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Building Packages with Colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS2 Actions](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html)

### Python & Development

- [Python 3.8 Documentation](https://docs.python.org/3.8/)
- [ROS2 Python Client Library](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Built with ❤️ using ROS2 Foxy and Python 3.8**

