# Milestone 6



## Helpful Guides

### Foxy
- [Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Building Packages](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

## Development Environment

### Setup

1. Clone the repository
```sh
git clone https://github.com/CS7389K/Milestone-6.git
cd Milestone-6
```

2. ROS2 Foxy requires Ubuntu 20.04, so ensure it's what you're using. If you're using windows, run the following to use WSL:
```sh
wsl --install -d Ubuntu-20.04
```

If you need to move WSL to a different drive (e.g. the F drive):
```
wsl --manage Ubuntu-20.04 --move F:\WSL
```

3. Install Foxy
```sh
sh install-ros2-foxy-desktop.sh
```

### Building the Project

Ensure you are in the project's root directory, then run:

```sh
colcon build --symlink-install
. install/setup.sh
```