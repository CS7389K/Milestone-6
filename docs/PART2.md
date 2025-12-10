# Part 2: Complete Object Grab and Transport Mission

## Overview

This implementation completes the Part 2 requirement: **autonomous object grabbing and 1-meter transport**.

The robot will:
1. **Detect** target objects (bottles, cups, etc.) using YOLO object detection
2. **Center** on the object by rotating the base
3. **Approach** the object by moving forward to optimal grabbing distance
4. **Grab** the object using the OpenManipulator-X arm and gripper
5. **Transport** the object forward by 1 meter (3.2 feet)
6. **Release** the object and return to home position

## Architecture

### Key Components

1. **`part2.py`** - Main mission controller implementing the complete state machine
   - Uses `FollowJointTrajectory` action for precise arm control
   - Implements object centering based on YOLO detections
   - Coordinates base movement and arm manipulation

2. **`yolo_publisher.py`** - Object detection node
   - Publishes real-time object detections with bounding box data
   - Provides visual feedback window (optional)

3. **`TeleopPublisher`** - Unified control interface
   - Sends base velocity commands
   - Sends arm trajectory goals
   - Controls gripper actions

### State Machine

```
IDLE
  ↓ (object detected)
CENTERING
  ↓ (object centered in frame)
APPROACHING
  ↓ (at optimal grab distance)
GRABBING
  ├─ Open gripper
  ├─ Position arm
  ├─ Extend to grasp
  ├─ Close gripper
  └─ Lift object
  ↓ (object grabbed)
TRANSPORTING
  ↓ (moved 1 meter forward)
RELEASING
  ├─ Lower arm
  ├─ Open gripper
  └─ Retract to home
  ↓
DONE → IDLE
```

## Usage


**Step 1: Start hardware controllers (on TurtleBot)**
```bash
# Source ROS2 (if not already sourced)
source /opt/ros/foxy/setup.bash

# Start hardware
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

**Step 2: Start (on Remote PC or TurtleBot)**
```bash
# Source workspace
cd ~/Desktop/Milestone-6
source install/setup.bash

# Launch Part 2
ros2 launch milestone6 part2.launch.py
```

**Step 3: Place target object at edge of camera view**
- The robot will automatically detect and execute the mission
- Monitor progress in the terminal output

## Launch Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tracking_classes` | string | '39' | Comma-separated COCO class IDs (39=bottle, 41=cup) |
| `image_width` | int | 1280 | Camera image width (pixels) |
| `image_height` | int | 720 | Camera image height (pixels) |
| `bbox_tolerance` | int | 20 | Acceptable bbox width range (pixels) |
| `center_tolerance` | int | 30 | Centering tolerance (pixels) |
| `target_bbox_width` | int | 180 | Ideal bbox width for grabbing (pixels) |
| `forward_speed` | float | 0.15 | Linear velocity (m/s) |
| `turn_speed` | float | 1.0 | Angular velocity (rad/s) |
| `detection_timeout` | float | 0.5 | Detection timeout (seconds) |
| `transport_distance` | float | 1.0 | Transport distance (meters) |

### Examples

```bash
# Track cups instead of bottles
ros2 launch milestone6 part2.launch.py tracking_classes:='41'

# Track both bottles and cups
ros2 launch milestone6 part2.launch.py tracking_classes:='39,41'

# Disable YOLO display
ros2 launch milestone6 part2.launch.py display:=false

# Transport 0.5m instead of 1m
ros2 launch milestone6 part2.launch.py transport_distance:=0.5

# Adjust speeds
ros2 launch milestone6 part2.launch.py forward_speed:=0.2 turn_speed:=1.2

# Fine-tune centering
ros2 launch milestone6 part2.launch.py center_tolerance:=20 target_bbox_width:=200
```

## Troubleshooting

### Common Issues

**1. "Arm controller not available!"**
```bash
# Solution: Ensure hardware is running
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

# Verify controllers
ros2 topic list | grep arm_controller
ros2 topic echo /joint_states
```

**2. "No joint states available!"**
```bash
# Check if joint_states topic exists
ros2 topic hz /joint_states

# If not, restart hardware controllers
```

**3. Robot doesn't detect target object**
```bash
# Check YOLO output
ros2 topic echo /yolo_topic

# Verify camera
ros2 run milestone6 yolo_publisher --ros-args -p display:=true

# Check if correct class is being tracked
ros2 param get /part2 tracking_classes

# Check lighting and object visibility
```

**4. Robot moves too fast/slow**
```bash
# Adjust speeds
ros2 launch milestone6 part2.launch.py \
  forward_speed:=0.1 \
  turn_speed:=0.8
```

**5. Centering is imprecise**
```bash
# Reduce tolerance for tighter centering
ros2 launch milestone6 part2.launch.py \
  center_tolerance:=20
```

### Debugging Tips

**Check YOLO detections:**
```bash
ros2 topic echo /yolo_topic
```

**Monitor arm positions:**
```bash
ros2 topic echo /joint_states
```

**Verify velocity commands:**
```bash
ros2 topic echo /cmd_vel
```
