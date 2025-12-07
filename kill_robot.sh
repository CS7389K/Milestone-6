#!/bin/bash
# Script to stop all TurtleBot3 hardware and processes

echo "=== Stopping TurtleBot3 Hardware and Processes ==="
echo ""

# 1. Kill all ROS2 processes
echo "Killing all ROS2 processes..."
pkill -9 -f ros2
pkill -9 -f turtlebot3
pkill -9 -f robot_state_publisher
pkill -9 -f controller_manager
pkill -9 -f joint_state_publisher
pkill -9 -f gazebo

# 2. Stop systemd services
echo "Stopping systemd services..."
sudo systemctl stop turtlebot3-core 2>/dev/null
sudo systemctl stop robot 2>/dev/null

# 3. Send stop command to OpenCR board
echo "Sending stop command to OpenCR..."
# Find the OpenCR USB device
OPENCR_PORT=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)

if [ -n "$OPENCR_PORT" ]; then
    echo "Found OpenCR at $OPENCR_PORT"
    # Send zero velocity commands
    python3 -c "
import serial
import struct
import time
try:
    ser = serial.Serial('$OPENCR_PORT', 115200, timeout=1)
    time.sleep(0.1)
    # Send stop command multiple times
    for _ in range(10):
        # This sends zero velocities
        ser.write(b'\xFF\xFF\xFD\x00\x00\x00\x00\x00')
        time.sleep(0.05)
    ser.close()
    print('Stop command sent to OpenCR')
except Exception as e:
    print(f'Could not send stop command: {e}')
" 2>/dev/null || echo "Python serial not available, skipping direct stop command"
    
    # Alternative: reset the USB device
    echo "Resetting OpenCR USB connection..."
    OPENCR_BUS=$(udevadm info -q path -n $OPENCR_PORT | cut -d'/' -f6)
    if [ -n "$OPENCR_BUS" ]; then
        sudo sh -c "echo 0 > /sys/bus/usb/devices/$OPENCR_BUS/authorized" 2>/dev/null
        sleep 0.5
        sudo sh -c "echo 1 > /sys/bus/usb/devices/$OPENCR_BUS/authorized" 2>/dev/null
    fi
else
    echo "OpenCR port not found"
fi

# 4. Final cleanup
echo ""
echo "Cleanup complete."
echo ""
echo "Remaining ROS2 processes:"
ps aux | grep -E "ros2|turtlebot3" | grep -v grep | grep -v kill_robot || echo "None found."
echo ""
echo "If robot is STILL moving, physically press the RESET button on OpenCR board"
echo "or power cycle the robot."
