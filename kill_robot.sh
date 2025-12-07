#!/bin/bash
# Script to stop all ROS2 nodes and processes related to TurtleBot3

echo "Stopping all ROS2 processes forcefully..."

# Kill with increasing force: TERM -> KILL
pkill -9 -f "ros2"
pkill -9 -f "turtlebot3"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "joint_state_publisher"
pkill -9 -f "controller_manager"
pkill -9 -f "gazebo"
pkill -9 -f "gzserver"
pkill -9 -f "gzclient"
pkill -9 -f "python3.*milestone"
pkill -9 -f "opencr"

# Kill any OpenCR hardware interface
sudo pkill -9 -f "opencr"

# Stop any systemd services related to TurtleBot3
if systemctl list-units --type=service | grep -q turtlebot3; then
    echo "Stopping turtlebot3 systemd services..."
    sudo systemctl stop turtlebot3* 2>/dev/null
fi

# Reset USB connections (this disconnects OpenCR board)
echo "Resetting USB connections..."
for device in /sys/bus/usb/devices/*/authorized; do
    if [ -w "$device" ]; then
        sudo sh -c "echo 0 > $device"
        sudo sh -c "echo 1 > $device"
    fi
done

echo ""
echo "All processes killed and hardware reset."
echo ""
echo "Remaining ROS2 processes:"
ps aux | grep -E "ros2|turtlebot3" | grep -v grep || echo "None found."
