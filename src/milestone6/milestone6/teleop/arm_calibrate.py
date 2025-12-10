#!/usr/bin/env python3
"""
arm_calibrate.py

Interactive calibration tool for arm reaching poses.
Allows real-time tuning of joint angles while observing the arm position.

Usage:
    ros2 run milestone6 arm_calibrate

Controls:
    1/2: Decrease/Increase joint1 (base rotation)
    q/w: Decrease/Increase joint2 (shoulder - forward/down)
    a/s: Decrease/Increase joint3 (elbow - extend/retract)
    z/x: Decrease/Increase joint4 (wrist)
    
    g: Open gripper
    h: Close gripper
    
    r: Reset to home position
    p: Print current joint positions
    
    SPACE: Save current pose as preset
    ESC/Ctrl+C: Exit

The goal is to find the right joint2/joint3 values that:
1. Reach forward to the bottle
2. Position gripper at bottle height
3. Don't topple the bottle over
"""

import sys
import termios
import tty
import math

import rclpy
from rclpy.node import Node

from milestone6.teleop.publisher import TeleopPublisher
from milestone6.teleop.subscriber import TeleopSubscriber


class ArmCalibrator(Node):
    def __init__(self):
        super().__init__('arm_calibrator')
        
        self.get_logger().info("Starting Arm Calibrator...")
        
        # Publishers/Subscribers
        self.pub = TeleopPublisher(self)
        self.sub = TeleopSubscriber(self)
        
        # Current target pose
        self.target_pose = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0
        }
        
        # Step sizes
        self.step_small = 0.05  # 2.86 degrees
        self.step_large = 0.1   # 5.73 degrees
        
        # Saved presets
        self.presets = {
            'home': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0},
            'close': {},
            'medium': {},
            'far': {}
        }
        
        self.get_logger().info("Arm Calibrator ready!")
        self.print_help()
        
    def print_help(self):
        print("\n" + "="*70)
        print("ARM CALIBRATION CONTROLS")
        print("="*70)
        print("JOINT CONTROLS (fine: lowercase, coarse: uppercase):")
        print("  1/2 (or !/@): joint1 (base rotation, left/right)")
        print("  q/w (or Q/W): joint2 (shoulder, UP/DOWN-FORWARD)")
        print("  a/s (or A/S): joint3 (elbow, RETRACT/EXTEND)")
        print("  z/x (or Z/X): joint4 (wrist)")
        print()
        print("GRIPPER:")
        print("  g: Open gripper")
        print("  h: Close gripper")
        print()
        print("PRESETS:")
        print("  r: Reset to home [0, 0, 0, 0]")
        print("  [: Load 'close' preset")
        print("  ]: Load 'far' preset")
        print()
        print("UTILITIES:")
        print("  p: Print current joint positions")
        print("  P: Print and SAVE current as preset")
        print("  ESC/Ctrl+C: Exit")
        print("="*70)
        print("\nTIP: Start at home, then adjust joint2 (down) and joint3 (extend)")
        print("     to reach toward the bottle without toppling it.")
        print()
    
    def move_to_pose(self, pose):
        """Send arm to target pose."""
        self.target_pose = dict(pose)
        self.pub.send_arm_trajectory(pose)
        self.print_status()
    
    def print_status(self):
        """Print current pose."""
        j1 = self.target_pose['joint1']
        j2 = self.target_pose['joint2']
        j3 = self.target_pose['joint3']
        j4 = self.target_pose['joint4']
        
        print(f"\rCurrent: j1={j1:+.3f} j2={j2:+.3f} j3={j3:+.3f} j4={j4:+.3f}  ", end='', flush=True)
    
    def adjust_joint(self, joint_name, delta):
        """Adjust a joint by delta and move."""
        self.target_pose[joint_name] = max(-3.0, min(3.0, self.target_pose[joint_name] + delta))
        self.move_to_pose(self.target_pose)
    
    def handle_key(self, key):
        """Handle keyboard input."""
        step = self.step_small if key.islower() else self.step_large
        
        if key in '1!':
            self.adjust_joint('joint1', -step)
        elif key in '2@':
            self.adjust_joint('joint1', +step)
        elif key in 'qQ':
            self.adjust_joint('joint2', -step)
        elif key in 'wW':
            self.adjust_joint('joint2', +step)
        elif key in 'aA':
            self.adjust_joint('joint3', -step)
        elif key in 'sS':
            self.adjust_joint('joint3', +step)
        elif key in 'zZ':
            self.adjust_joint('joint4', -step)
        elif key in 'xX':
            self.adjust_joint('joint4', +step)
        
        elif key == 'g':
            self.pub.gripper_open()
            print("\n[GRIPPER OPENED]")
        elif key == 'h':
            self.pub.gripper_close()
            print("\n[GRIPPER CLOSED]")
        
        elif key == 'r':
            self.move_to_pose(self.presets['home'])
            print("\n[RESET TO HOME]")
        elif key == '[':
            if self.presets['close']:
                self.move_to_pose(self.presets['close'])
                print("\n[LOADED 'close' PRESET]")
        elif key == ']':
            if self.presets['far']:
                self.move_to_pose(self.presets['far'])
                print("\n[LOADED 'far' PRESET]")
        
        elif key == 'p':
            print(f"\n\nCurrent pose: {self.target_pose}")
            actual = self.sub.joint_positions if self.sub.have_joint_states else "N/A"
            print(f"Actual joints: {actual}\n")
        elif key == 'P':
            print(f"\n\n{'='*70}")
            print("SAVE THIS POSE AS PRESET:")
            print(f"    'joint1': {self.target_pose['joint1']:.3f},")
            print(f"    'joint2': {self.target_pose['joint2']:.3f},")
            print(f"    'joint3': {self.target_pose['joint3']:.3f},")
            print(f"    'joint4': {self.target_pose['joint4']:.3f}")
            print("="*70)
            print("\nCopy these values into arm1.py for your distance preset!\n")
        
        elif key == '\x1b':  # ESC
            return False
        
        return True


def get_key():
    """Get a single keypress without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    calibrator = ArmCalibrator()
    
    print("\nWaiting for arm to initialize...")
    rclpy.spin_once(calibrator, timeout_sec=2.0)
    
    print("Ready! Press keys to move arm...\n")
    
    try:
        while True:
            rclpy.spin_once(calibrator, timeout_sec=0.01)
            
            # Check for keyboard input (non-blocking)
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                key = get_key()
                if not calibrator.handle_key(key):
                    break
    
    except KeyboardInterrupt:
        print("\n\nCalibration ended.")
    
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
