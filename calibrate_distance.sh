#!/bin/bash
# Distance Calibration Workflow for TurtleBot3 Manipulation
#
# This script helps you find the optimal bbox_width (in pixels) where
# the arm can successfully grab the object.
#
# Usage:
#   1. Place a bottle in front of the robot
#   2. Run: ./calibrate_distance.sh
#   3. Watch the bbox_width values in YOLO output
#   4. Manually move robot closer/farther until arm can grab
#   5. Note the bbox_width at successful grab position
#   6. Update target_bbox_width parameter in part2.launch.py

echo "=========================================="
echo "Distance Calibration Helper"
echo "=========================================="
echo ""
echo "GOAL: Find the bbox_width (pixels) at optimal grab distance"
echo ""
echo "STEPS:"
echo "  1. Make sure hardware.launch.py is running in Terminal 1"
echo "  2. This script will show live YOLO detections"
echo "  3. Manually drive robot toward bottle using keyboard teleop:"
echo "     - Open new terminal"
echo "     - Run: ros2 run turtlebot3_teleop teleop_keyboard"
echo "     - Drive forward/backward until arm reaches bottle"
echo "  4. Note the bbox_width when arm successfully grabs"
echo "  5. Press Ctrl+C here when done"
echo ""
echo "Starting in 3 seconds..."
sleep 3

echo ""
echo "=========================================="
echo "Monitoring /yolo/data topic for bbox_width..."
echo "=========================================="
echo ""

# Monitor YOLO detections and extract bbox_width
ros2 topic echo /yolo/data --field bbox_w

# After user stops with Ctrl+C
echo ""
echo "=========================================="
echo "Calibration Notes:"
echo "=========================================="
echo ""
echo "The bbox_width you saw at successful grab position should be"
echo "set as 'target_bbox_width' in part2.launch.py"
echo ""
echo "Typical values:"
echo "  - Very close (~20cm): 250-300px"
echo "  - Close (~25cm): 180-220px"  
echo "  - Medium (~30cm): 130-170px"
echo "  - Far (~40cm): 90-120px"
echo ""
echo "IMPORTANT: The arm can only reach ~30-35cm max!"
echo "           Choose a bbox_width in the 150-200px range."
echo ""
