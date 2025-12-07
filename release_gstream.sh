#!/bin/bash
# Script to release GStreamer and camera resources
# Run this if the camera fails to open due to "resource busy" errors

echo "Releasing GStreamer and camera resources..."

# Kill any hanging GStreamer processes
echo "1. Killing GStreamer processes..."
pkill -9 gst-launch-1.0 2>/dev/null
pkill -9 nvargus-daemon 2>/dev/null

# Restart the NVIDIA Argus camera daemon
echo "2. Restarting nvargus-daemon..."
sudo systemctl restart nvargus-daemon

# Wait for daemon to fully restart
echo "3. Waiting for daemon to initialize..."
sleep 2

# Check status
echo "4. Checking nvargus-daemon status..."
sudo systemctl status nvargus-daemon --no-pager | head -n 5

echo ""
echo "Camera resources released!"
echo "You can now try running your program again."
