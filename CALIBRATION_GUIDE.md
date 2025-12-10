# Complete Calibration Guide for TurtleBot3 Manipulation

This guide helps you calibrate the system so the robot can consistently grab bottles.

## Problem

The system has two parts that must work together:
1. **Base controller**: Drives robot to correct distance from object
2. **Arm controller**: Reaches and grabs object

Currently, they're not calibrated to each other!

---

## Solution: 2-Step Calibration Process

### Step 1: Find Optimal Approach Distance

**Goal**: Determine what bbox_width (pixels) corresponds to the distance where the arm CAN reach the object.

**Process**:
```bash
# Terminal 1: Hardware
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

# Terminal 2: Manual teleop
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 3: Monitor bbox size
ros2 topic echo /yolo/data --field bbox_w

# Terminal 4: Test arm reach manually
ros2 run milestone6 arm_calibrate
```

**Workflow**:
1. Place bottle 30cm in front of robot
2. Look at Terminal 3 - note the bbox_width (e.g., "185")
3. In Terminal 4 (arm_calibrate):
   - Press `r` to reset to home
   - Press `w` to reach forward (increase joint2)
   - Press `s` to extend (increase joint3)
   - Press `g`/`h` to test gripper position
4. If arm can't reach:
   - Use Terminal 2 to drive CLOSER
   - Note new bbox_width in Terminal 3
   - Retry arm calibration in Terminal 4
5. Find the distance where arm JUST reaches bottle
6. **Record that bbox_width!** This is your `target_bbox_width`

**Example**:
```
Robot 40cm away: bbox_width = 120px → ARM CAN'T REACH ✗
Robot 35cm away: bbox_width = 145px → ARM CAN'T REACH ✗  
Robot 28cm away: bbox_width = 180px → ARM CAN REACH ✓ ← USE THIS!
Robot 20cm away: bbox_width = 240px → TOO CLOSE (topples bottle) ✗
```

### Step 2: Calibrate Arm Pose for That Distance

Now that you know the target distance (e.g., bbox_width=180px), calibrate the arm pose:

**Process**:
```bash
# Terminal 1: Hardware (still running)
# Terminal 2: Position robot at target distance manually

# Terminal 3: Monitor that you're at correct distance
ros2 topic echo /yolo/data --field bbox_w
# Should show ~180px (your target from Step 1)

# Terminal 4: Calibrate arm
ros2 run milestone6 arm_calibrate
```

**Workflow**:
1. Ensure robot is at target distance (bbox_width ≈ 180px)
2. In arm_calibrate:
   ```
   Press r - reset to home [0, 0, 0, 0]
   Press w (5-10 times) - reach down to bottle height
   Press s (3-6 times) - extend forward toward bottle
   Press g - open gripper
   Check - is gripper at bottle, not toppling it?
   ```
3. Adjust until gripper is JUST touching bottle
4. Press `P` to print values
5. Copy those values

**Example output**:
```
SAVE THIS POSE AS PRESET:
    'joint1': 0.000,
    'joint2': 0.450,
    'joint3': -0.250,
    'joint4': 0.000
```

---

## Step 3: Update Code with Calibrated Values

### Update base.py target distance:

Edit `src/milestone6/launch/part2.launch.py`:
```python
base_node = Node(
    package='milestone6',
    executable='base',
    name='teleop_base',
    output='screen',
    parameters=[{
        'move_wheels': True,
        'image_width': 500,
        'image_height': 320,
        'tolerance': 50,
        'target_bbox_width': 180,  # ← YOUR VALUE FROM STEP 1
        'bbox_tolerance': 15,       # Allow ±15px variation
        'forward_speed': 0.15,
        'turn_speed': 1.5,
        'track_class': 39,
    }]
)
```

### Update arm1.py with calibrated poses:

Edit `src/milestone6/milestone6/teleop/arm1.py` around line 175:
```python
# Replace the distance presets with your calibrated values
if bbox_width_normalized > 0.35:  # Very close
    joint2 = 0.300  # ← Adjust based on calibration
    joint3 = -0.150
    joint4 = 0.0
elif bbox_width_normalized > 0.25:  # Medium (YOUR TARGET DISTANCE)
    joint2 = 0.450  # ← YOUR VALUES FROM STEP 2
    joint3 = -0.250  # ← YOUR VALUES FROM STEP 2
    joint4 = 0.0
else:  # Far
    joint2 = 0.600
    joint3 = -0.350
    joint4 = 0.0
```

---

## Step 4: Test Full Autonomous System

```bash
cd ~/Milestone6/Milestone-6
colcon build --packages-select milestone6
source install/setup.bash

# Terminal 1: Hardware
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

# Terminal 2: Autonomous system
./run_part2.sh
```

**Expected behavior**:
1. Robot detects bottle
2. Drives forward/backward until bbox_width ≈ 180px (your target)
3. Stops and prints "✓ At target distance!"
4. Arm reaches forward to calibrated pose
5. Gripper closes on bottle
6. Arm lifts bottle

---

## Troubleshooting

**Robot doesn't stop at right distance:**
- Check `target_bbox_width` in launch file
- Increase `bbox_tolerance` if it's oscillating

**Arm doesn't reach bottle:**
- Target distance is too far, reduce `target_bbox_width`
- Re-run Step 1 to find closer distance

**Arm topples bottle:**
- Target distance is too close, increase `target_bbox_width`
- Or reduce joint2 value in arm calibration

**Gripper misses bottle (too high/low):**
- Re-run Step 2 arm calibration
- Adjust joint2 (up/down) in the code

---

## Quick Reference

**Typical Calibrated Values** (for reference, yours may differ):
```
target_bbox_width: 180px (≈28cm distance)
joint2: 0.45 (reach down)
joint3: -0.25 (extend forward)
bbox_tolerance: 15px
```

**Key Insight**: The robot MUST approach to a consistent distance (controlled by base), THEN the arm can use a pre-calibrated pose (controlled by arm) to grab reliably.
