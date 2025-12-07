#!/bin/bash
# Script to stop all ROS2 nodes and processes related to TurtleBot3

echo "Stopping all ROS2 nodes..."

# Kill all ros2 launch processes
pkill -f "ros2 launch"

# Kill all ros2 run processes
pkill -f "ros2 run"

# Kill specific TurtleBot3 and manipulation processes
pkill -f "turtlebot3"
pkill -f "robot_state_publisher"
pkill -f "joint_state_publisher"
pkill -f "controller_manager"
pkill -f "hardware.launch"
pkill -f "robot.launch"

# Kill any remaining ROS2 daemon
pkill -f "ros2 daemon"

# Kill gazebo if running
pkill -f "gazebo"
pkill -f "gzserver"
pkill -f "gzclient"

echo "All ROS2 processes killed."
echo ""
echo "To verify, run: ps aux | grep ros2"
