#!/bin/bash
# Wrapper script to fix PyTorch TLS memory allocation issue
# This preloads libgomp before PyTorch loads, preventing the TLS block error

# Find libgomp.so.1 on the system
LIBGOMP_PATH=$(find /usr/lib -name "libgomp.so.1" 2>/dev/null | head -1)

if [ -z "$LIBGOMP_PATH" ]; then
    echo "Warning: libgomp.so.1 not found, trying alternative method..."
    # Try system library path directly
    export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
else
    echo "Found libgomp at: $LIBGOMP_PATH"
    export LD_PRELOAD=$LIBGOMP_PATH
fi

# Source ROS2 setup
source /opt/ros/foxy/setup.bash
source ~/Milestone6/Milestone-6/install/setup.bash

# Launch the system
echo "Launching part2 with LD_PRELOAD=$LD_PRELOAD"
ros2 launch milestone6 part2.launch.py "$@"
