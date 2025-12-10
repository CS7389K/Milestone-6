#!/bin/bash
# Script to find optimal bbox_width for manipulation

echo "=========================================="
echo "Finding Optimal Bbox Width for New Camera"
echo "=========================================="
echo ""
echo "Instructions:"
echo "1. Place bottle in front of robot"
echo "2. Manually drive robot to a distance where you think arm can reach"
echo "3. Look at the YOLO detection window - note the bbox width"
echo "4. Run: ros2 run milestone6 arm_calibrate"
echo "5. Use arm_calibrate to test if arm can reach at that distance"
echo "6. If successful, record that bbox_width value"
echo ""
echo "Starting YOLO detection to show bbox sizes..."
echo ""

cd ~/Desktop/Milestone-6
source install/setup.bash

# Launch just YOLO publisher to see detections
ros2 run milestone6 yolo_publisher --ros-args -p display:=true
