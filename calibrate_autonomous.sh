#!/bin/bash
# Simple Calibration Workflow - Autonomous Base + Manual Arm Tuning
#
# The base will autonomously drive to the target distance (bbox_width=180px).
# You only need to calibrate the arm pose at that distance.

echo "=========================================="
echo "Autonomous Manipulation Calibration"
echo "=========================================="
echo ""
echo "STEP 1: Find the right target distance"
echo "----------------------------------------"
echo ""
echo "The base is currently configured to stop at bbox_width=180px."
echo "We need to verify the arm can reach at that distance."
echo ""
read -p "Press Enter to start autonomous approach test..."

# Check if hardware is running
if ! ros2 topic list | grep -q "/joint_states"; then
    echo ""
    echo "ERROR: Hardware not running!"
    echo "Please start in another terminal:"
    echo "  ros2 launch turtlebot3_manipulation_bringup hardware.launch.py"
    echo ""
    exit 1
fi

echo ""
echo "Starting autonomous base (will drive to bottle and stop)..."
echo "Place a bottle in front of the robot and watch..."
echo ""

# Start base node in background
ros2 run milestone6 base --ros-args -p move_wheels:=true -p target_bbox_width:=180 -p bbox_tolerance:=15 -p track_class:=39 &
BASE_PID=$!

# Start YOLO publisher in background
ros2 run milestone6 yolo_publisher &
YOLO_PID=$!

sleep 3
echo ""
echo "Base is running. Watch for message: '✓ At target distance!'"
echo "When it stops, the robot is at the calibrated approach distance."
echo ""
echo "Press Ctrl+C when base has stopped at target..."

# Wait for user to stop
trap "kill $BASE_PID $YOLO_PID 2>/dev/null; exit" INT
wait

echo ""
echo "=========================================="
echo "STEP 2: Calibrate arm pose"
echo "=========================================="
echo ""
echo "Now we'll tune the arm to reach the bottle at this distance."
echo ""
read -p "Press Enter to start arm calibration tool..."

# Launch arm calibrator
ros2 run milestone6 arm_calibrate

echo ""
echo "=========================================="
echo "STEP 3: Update code with calibrated values"
echo "=========================================="
echo ""
echo "Copy the joint values you found into:"
echo "  src/milestone6/milestone6/teleop/arm1.py"
echo ""
echo "Look for the 'CLOSE' preset (around line 187) and update:"
echo "  elif bbox_width_normalized > 0.25:  # Close"
echo "      joint2 = YOUR_VALUE"
echo "      joint3 = YOUR_VALUE"
echo ""
echo "Then rebuild:"
echo "  colcon build --packages-select milestone6"
echo "  source install/setup.bash"
echo ""
echo "And test the full autonomous system:"
echo "  ./run_part2.sh"
echo ""
