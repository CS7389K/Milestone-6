#!/bin/bash
# Wrapper script to fix PyTorch TLS memory allocation issue
# This preloads PyTorch's libgomp before other libraries load

# Find PyTorch's bundled libgomp (the one causing the error)
TORCH_LIBGOMP=$(find ~/.local/lib/python3.8/site-packages/torch.libs -name "libgomp*.so*" 2>/dev/null | head -1)

if [ -z "$TORCH_LIBGOMP" ]; then
    echo "Warning: PyTorch libgomp not found, trying system libgomp..."
    # Fallback to system library
    TORCH_LIBGOMP=$(find /usr/lib -name "libgomp.so.1" 2>/dev/null | head -1)
fi

if [ -n "$TORCH_LIBGOMP" ]; then
    echo "Found libgomp at: $TORCH_LIBGOMP"
    export LD_PRELOAD=$TORCH_LIBGOMP
else
    echo "ERROR: Could not find libgomp! PyTorch may fail to load."
fi

# Source ROS2 setup
source /opt/ros/foxy/setup.bash
source ~/Milestone6/Milestone-6/install/setup.bash

# Launch the system
echo "Launching part2 with LD_PRELOAD=$LD_PRELOAD"
ros2 launch milestone6 part2.launch.py "$@"
