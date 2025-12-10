#!/bin/bash
# Quick diagnostic script to check if required topics are available

echo "=========================================="
echo "ROS2 Topic Diagnostics"
echo "=========================================="
echo ""

echo "Checking for /joint_states topic..."
if ros2 topic list | grep -q "/joint_states"; then
    echo "✓ /joint_states topic EXISTS"
    echo "  Message count:"
    timeout 2s ros2 topic hz /joint_states 2>&1 | head -5 || echo "  (no messages being published)"
else
    echo "✗ /joint_states topic NOT FOUND"
    echo "  → You need to run hardware.launch.py first!"
fi

echo ""
echo "Checking for /yolo/data topic..."
if ros2 topic list | grep -q "/yolo/data"; then
    echo "✓ /yolo/data topic EXISTS"
    echo "  Message count:"
    timeout 2s ros2 topic hz /yolo/data 2>&1 | head -5 || echo "  (no messages being published)"
else
    echo "✗ /yolo/data topic NOT FOUND"
    echo "  → YOLO publisher may not be running"
fi

echo ""
echo "Checking for /cmd_vel topic..."
if ros2 topic list | grep -q "/cmd_vel"; then
    echo "✓ /cmd_vel topic EXISTS"
else
    echo "✗ /cmd_vel topic NOT FOUND"
fi

echo ""
echo "=========================================="
echo "All Topics:"
echo "=========================================="
ros2 topic list

echo ""
echo "=========================================="
echo "Active Nodes:"
echo "=========================================="
ros2 node list
