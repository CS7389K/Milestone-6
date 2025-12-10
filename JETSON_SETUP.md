# 🤖 Jetson Setup Guide for Milestone 6

## 📋 Quick Start

Follow these steps to get your TurtleBot3 with OpenMANIPULATOR-X running on the Jetson.

---

## 1️⃣ Initial Setup (One-Time)

### Navigate to the Project
```bash
cd ~/Milestone6  # Or wherever you cloned it
```

### Install ROS2 Foxy (if not already installed)
```bash
# Check if ROS2 is already installed
ros2 --version

# If not installed, run the installer
bash install-ros2-foxy-desktop.sh
```

### Install YOLO Model
```bash
cd models
bash yolo11n_install.sh
cd ..
```

### Install Python Dependencies
```bash
# Make sure you have pip
sudo apt update
sudo apt install python3-pip -y

# Install required Python packages
pip3 install opencv-python numpy ultralytics
```

### Build the Workspace
```bash
# Build only the milestone6 package
colcon build --packages-select milestone6 --symlink-install

# Source the workspace
source install/setup.bash
```

**💡 Tip:** Add this to your `~/.bashrc` so it's automatic:
```bash
echo "source ~/Milestone6/install/setup.bash" >> ~/.bashrc
```

---

## 2️⃣ Running the System

### Setup Terminal Environment (Each New Terminal)
```bash
cd ~/Milestone6
source install/setup.bash
```

### Terminal 1: Launch TurtleBot3 Hardware
```bash
# This starts the OpenMANIPULATOR-X arm and base controllers
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

**Expected Output:**
- ✅ Controller manager started
- ✅ Joint state broadcaster active
- ✅ Arm controller ready
- ✅ Gripper controller ready
- ✅ Diff drive controller ready

---

### Terminal 2A: Part 1 - Object Detection & Base Movement
```bash
source install/setup.bash
ros2 launch milestone6 part1.launch.py
```

**What This Does:**
- 🎥 Publishes YOLO object detections
- 🚗 Controls robot base to approach detected objects
- 📡 Publishes teleop control interface

---

### Terminal 2B: Part 2 - Full Autonomous Grabbing (YOUR FIXED CODE!)
```bash
source install/setup.bash
ros2 launch milestone6 part2.launch.py
```

**What This Does:**
- 🎥 YOLO object detection
- 🚗 Base movement to approach objects
- 🦾 **Autonomous arm control with adaptive grabbing**
- ✅ Full pick-and-place sequence with joint state feedback

**What to Expect:**
1. System waits for bottle/cup detection
2. When detected, logs: "TARGET DETECTED! class 39..."
3. Opens gripper
4. Moves arm to calculated position (adapts based on distance)
5. Logs: "Motion complete! Max error: 0.042 rad" when position reached
6. Approaches closer
7. Closes gripper
8. Lifts object
9. Holds for 2 seconds
10. Releases object

---

## 3️⃣ Monitoring & Debugging

### Check Joint States
```bash
# In a new terminal
ros2 topic echo /joint_states
```

### Check YOLO Detections
```bash
ros2 topic echo /yolo_detections
```

### Check Arm Commands
```bash
ros2 topic echo /arm_controller/follow_joint_trajectory/goal
```

### View All Active Topics
```bash
ros2 topic list
```

### View Node Information
```bash
# See what nodes are running
ros2 node list

# Get info about the arm node
ros2 node info /part2_mission
```

---

## 4️⃣ Troubleshooting

### Problem: "No joint states available"
**Solution:** Make sure Terminal 1 (hardware.launch.py) is running first

### Problem: "FollowJointTrajectory action server not ready"
**Solution:** 
```bash
# Check if controllers are loaded
ros2 control list_controllers

# Should see:
# - joint_state_broadcaster [active]
# - arm_controller [active]
# - gripper_action_controller [active]
# - diff_drive_controller [active]
```

### Problem: YOLO not detecting objects
**Solution:**
```bash
# Check camera feed
ros2 topic echo /camera/image_raw --once

# Check if YOLO model exists
ls ~/Milestone6/models/yolov11n.hef
```

### Problem: Arm not moving
**Check logs:**
```bash
# The arm node provides detailed feedback:
# - "Object CLOSE (bbox=180px), using moderate reach"
# - "Motion complete! Max error: 0.042 rad"
# - "Moving... Max error: 0.250 rad (tolerance: 0.150)"
```

**Verify joint limits:**
```bash
# Make sure joints aren't at limits
ros2 topic echo /joint_states | grep position
```

---

## 5️⃣ Testing Individual Components

### Test Gripper Only
```bash
# Open gripper
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.01, max_effort: -1.0}}"

# Close gripper
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: -0.01, max_effort: -1.0}}"
```

### Test Arm Movement
```bash
# Move to home position
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint1', 'joint2', 'joint3', 'joint4'], \
   points: [{positions: [0.0, -1.05, 0.35, 0.70], time_from_start: {sec: 2}}]}}"
```

---

## 6️⃣ Quick Reference

### Key Topics
- `/joint_states` - Current arm/gripper positions
- `/yolo_detections` - Object detection results
- `/cmd_vel` - Base velocity commands
- `/arm_controller/follow_joint_trajectory/goal` - Arm trajectory goals

### Key Parameters (can be modified in launch files)
- `target_class: 39` - COCO class for bottles
- `image_width: 500` - Camera resolution
- `detection_timeout_sec: 1.0` - How long to wait for fresh detections

### Validated Joint Poses (from ROBOTIS)
- **Home:** `[0.0, -1.05, 0.35, 0.70]` - Forward reach position
- **Init:** `[0.0, -1.57, 1.37, 0.26]` - Alternative init pose
- **Zero:** `[0.0, 0.0, 0.0, 0.0]` - All joints at zero

---

## 7️⃣ Shutdown Sequence

1. **Stop part2_mission:** `Ctrl+C` in Terminal 2
2. **Stop hardware:** `Ctrl+C` in Terminal 1
3. Wait for controllers to safely shut down

---

## 🎯 What Makes This System Work

Your **fixed implementation** now:

✅ **Waits for motion completion** using joint state feedback
✅ **Adapts reach distance** based on object bbox size
✅ **Uses ROBOTIS-validated poses** as foundation
✅ **Has safety timeouts** (5s) to prevent hanging
✅ **Step-by-step execution** with verification at each stage
✅ **Detailed logging** showing current vs target positions

The key improvement: Instead of blindly sending commands and hoping they work, the system now **actively monitors joint states** and waits until the arm has actually reached each target position before proceeding!

---

## 📞 Need Help?

Check logs with:
```bash
ros2 launch milestone6 part2.launch.py 2>&1 | tee robot_log.txt
```

This saves all output to `robot_log.txt` for debugging.

---

**Happy Grabbing! 🦾🎯**
