#!/bin/bash
# Script to release GStreamer and NVIDIA Argus camera resources on Jetson
# Run this if the camera fails to open due to "resource busy" errors

echo "Releasing NVIDIA Jetson camera resources..."

# Kill any Python processes using the camera
echo "1. Killing Python processes that may be holding the camera..."
pkill -9 python3 2>/dev/null
sleep 1

# Kill any ROS2 processes
echo "2. Killing ROS2 processes..."
pkill -9 ros2 2>/dev/null
sleep 1

# Kill GStreamer and OpenCV processes
echo "3. Killing GStreamer and OpenCV processes..."
pkill -9 gst-launch-1.0 2>/dev/null
pkill -9 nvarguscamerasrc 2>/dev/null
pkill -9 nvvidconv 2>/dev/null
sleep 1

# Kill and restart the NVIDIA Argus daemon (manages camera hardware)
echo "4. Stopping nvargus-daemon..."
sudo systemctl stop nvargus-daemon
sleep 1

echo "5. Killing any remaining nvargus processes..."
sudo pkill -9 nvargus-daemon 2>/dev/null
sudo pkill -9 nvcamera-daemon 2>/dev/null
sleep 1

# Restart the daemon
echo "6. Starting nvargus-daemon..."
sudo systemctl start nvargus-daemon
sleep 2

# Verify the camera device exists
echo "7. Checking camera device..."
if [ -c /dev/video0 ]; then
    echo "   ✓ Camera device /dev/video0 found"
else
    echo "   ✗ Warning: /dev/video0 not found"
fi

# Check daemon status
echo "8. Checking nvargus-daemon status..."
if sudo systemctl is-active --quiet nvargus-daemon; then
    echo "   ✓ nvargus-daemon is running"
else
    echo "   ✗ nvargus-daemon is NOT running"
    echo "   Try: sudo systemctl status nvargus-daemon"
fi

echo ""
echo "============================================"
echo "Camera resources released!"
echo "Wait 3-5 seconds before running your program."
echo "============================================"
