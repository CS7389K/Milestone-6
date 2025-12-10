# Quick Calibration Guide - Autonomous Base

The base controller is **fully autonomous** - it will:
1. Detect the bottle
2. Drive forward and rotate to center it
3. Stop at `target_bbox_width` (default 180px ≈ 28cm)
4. Signal when ready for arm to grab

You only need to **calibrate the arm pose** for that fixed distance.

---

## One-Time Calibration (5 minutes)

### Prerequisites
```bash
cd ~/Milestone6/Milestone-6
colcon build --packages-select milestone6
source install/setup.bash
chmod +x *.sh
```

### Terminal Setup

**Terminal 1 - Hardware** (keep running):
```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
```

**Terminal 2 - Autonomous approach test**:
```bash
# Place bottle somewhere in view
./run_part2.sh

# Watch for: "✓ At target distance! bbox=180px"
# The robot will drive to the bottle and stop automatically
```

**Terminal 3 - Arm calibration** (when robot stops):
```bash
# Robot is now at target distance
ros2 run milestone6 arm_calibrate

# Interactive controls:
#   r - reset to home [0, 0, 0, 0]
#   w/W - reach DOWN (increase joint2)
#   s/S - extend FORWARD (increase joint3)
#   g - open gripper
#   h - close gripper
#   P - print values when perfect
```

### Calibration Process

1. **Robot drives autonomously** to target distance (180px bbox)
2. **You tune arm pose**:
   ```
   Start: [0, 0, 0, 0]
   Press: w w w w w w → [0, 0.30, 0, 0]  (down)
   Press: s s s s → [0, 0.30, -0.20, 0]  (forward)
   Test: g (open), h (close) - check position
   Good? Press P to print
   ```
3. **Copy values** into `arm1.py` line ~187:
   ```python
   elif bbox_width_normalized > 0.25:  # Close (180px target)
       joint2 = 0.30   # YOUR CALIBRATED VALUE
       joint3 = -0.20  # YOUR CALIBRATED VALUE
       joint4 = 0.0
   ```
4. **Rebuild and test**:
   ```bash
   colcon build --packages-select milestone6
   source install/setup.bash
   ./run_part2.sh
   ```

---

## Adjusting Target Distance (if needed)

If arm can't reach at 180px, adjust `target_bbox_width`:

**Edit**: `src/milestone6/launch/part2.launch.py` line ~136:
```python
'target_bbox_width': 200,  # Increase = robot stops closer
'bbox_tolerance': 15,      # ±15px variation allowed
```

**Then recalibrate arm pose** at the new distance.

---

## Parameters Explained

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `target_bbox_width` | 180px | Stop when bottle is this wide (≈28cm) |
| `bbox_tolerance` | 15px | Allow 165-195px range |
| `forward_speed` | 0.12 | Slower for precise approach |
| `turn_speed` | 1.2 | Slower turning for stability |

**Larger bbox_width** = robot stops closer (but may topple object)
**Smaller bbox_width** = robot stops farther (arm may not reach)

---

## Testing

After calibration:
```bash
# Place bottle anywhere in front of robot
./run_part2.sh

# Expected:
# 1. Robot detects bottle
# 2. Drives forward, centering horizontally  
# 3. Stops at target distance: "✓ At target distance!"
# 4. Arm reaches with calibrated pose
# 5. Gripper closes on bottle
# 6. Arm lifts
```

---

## Troubleshooting

**Base doesn't stop / keeps oscillating:**
- Increase `bbox_tolerance` (try 20-25px)

**Arm doesn't reach bottle:**
- Increase `target_bbox_width` (try 200-220px)
- Recalibrate arm pose

**Arm topples bottle:**
- Decrease `target_bbox_width` (try 160-170px)  
- Or reduce joint2 in arm calibration

**Base stops but arm grabs air:**
- Your arm pose is miscalibrated
- Re-run arm_calibrate, adjust w/s carefully
