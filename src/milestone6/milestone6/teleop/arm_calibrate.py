#!/usr/bin/env python3
"""
arm_calibrate.py

Interactive real-time calibration tool for arm reaching poses.
Press keys to immediately move joints and see results.

Usage:
    ros2 run milestone6 arm_calibrate

Controls:
    1/2: Decrease/Increase joint1 (base rotation) - SMALL steps
    !/@: Decrease/Increase joint1 - LARGE steps
    
    q/w: Decrease/Increase joint2 (shoulder - forward/down) - SMALL
    Q/W: Decrease/Increase joint2 - LARGE
    
    a/s: Decrease/Increase joint3 (elbow - extend/retract) - SMALL  
    A/S: Decrease/Increase joint3 - LARGE
    
    z/x: Decrease/Increase joint4 (wrist) - SMALL
    Z/X: Decrease/Increase joint4 - LARGE
    
    g: Open gripper
    h: Close gripper
    
    r: Reset to home [0, 0, 0, 0]
    p: Print current position
    P: Print as code preset
    
    ESC: Exit

The arm moves IMMEDIATELY when you press a key (no Enter needed).
"""

import sys
import termios
import tty
import threading

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
        self.step_fine = 0.02    # 1.15 degrees - very precise
        self.step_medium = 0.05  # 2.86 degrees
        self.step_coarse = 0.10  # 5.73 degrees
        
        self.get_logger().info("Arm Calibrator ready!")
        self.print_help()
        
    def print_help(self):
        print("\n" + "="*70)
        print("REAL-TIME ARM CALIBRATION")
        print("="*70)
        print("Keys move arm IMMEDIATELY (no Enter needed)")
        print()
        print("FINE CONTROL (0.02 rad ≈ 1°):")
        print("  1/2: joint1 base (left/right)")
        print("  q/w: joint2 shoulder (UP/DOWN)")  
        print("  a/s: joint3 elbow (RETRACT/EXTEND)")
        print("  z/x: joint4 wrist")
        print()
        print("MEDIUM CONTROL (0.05 rad ≈ 3°) - Hold Shift:")
        print("  Q/W: joint2 shoulder")
        print("  A/S: joint3 elbow")
        print()
        print("COARSE CONTROL (0.10 rad ≈ 6°) - Symbols:")
        print("  !/@: joint1 base")
        print("  Z/X: joint4 wrist")
        print()
        print("GRIPPER:")
        print("  g: Open    h: Close")
        print()
        print("UTILITIES:")
        print("  r: Reset to home [0,0,0,0]")
        print("  p: Print current pose")
        print("  P: Print as code preset")
        print("  ESC: Exit")
        print("="*70)
        print("\nTIP: Start with 'r', then use w/s keys to find bottle position")
        print()
    
    def move_to_pose(self, pose):
        """Send arm to target pose immediately."""
        self.target_pose = dict(pose)
        self.pub.send_arm_trajectory(pose)
        self.print_status()
    
    def print_status(self):
        """Print current pose on same line."""
        j1 = self.target_pose['joint1']
        j2 = self.target_pose['joint2']
        j3 = self.target_pose['joint3']
        j4 = self.target_pose['joint4']
        
        # Show degrees for readability
        j1_deg = j1 * 57.3
        j2_deg = j2 * 57.3
        j3_deg = j3 * 57.3
        j4_deg = j4 * 57.3
        
        print(f"\rj1={j1:+.3f} ({j1_deg:+4.0f}°) | j2={j2:+.3f} ({j2_deg:+4.0f}°) | "
              f"j3={j3:+.3f} ({j3_deg:+4.0f}°) | j4={j4:+.3f} ({j4_deg:+4.0f}°)   ", 
              end='', flush=True)
    
    def adjust_joint(self, joint_name, delta):
        """Adjust a joint by delta and move immediately."""
        self.target_pose[joint_name] = max(-3.0, min(3.0, self.target_pose[joint_name] + delta))
        self.move_to_pose(self.target_pose)
    
    def handle_key(self, key):
        """Handle keyboard input with variable step sizes."""
        
        # Determine step size based on key case/symbol
        if key.islower():
            step = self.step_fine
        elif key.isupper():
            step = self.step_medium
        else:
            step = self.step_coarse
        
        # Joint 1 - Base rotation
        if key in '1':
            self.adjust_joint('joint1', -self.step_fine)
        elif key in '2':
            self.adjust_joint('joint1', +self.step_fine)
        elif key in '!':
            self.adjust_joint('joint1', -self.step_coarse)
        elif key in '@':
            self.adjust_joint('joint1', +self.step_coarse)
        
        # Joint 2 - Shoulder (up/down)
        elif key in 'q':
            self.adjust_joint('joint2', -self.step_fine)
        elif key in 'w':
            self.adjust_joint('joint2', +self.step_fine)
        elif key in 'Q':
            self.adjust_joint('joint2', -self.step_medium)
        elif key in 'W':
            self.adjust_joint('joint2', +self.step_medium)
        
        # Joint 3 - Elbow (extend/retract)
        elif key in 'a':
            self.adjust_joint('joint3', -self.step_fine)
        elif key in 's':
            self.adjust_joint('joint3', +self.step_fine)
        elif key in 'A':
            self.adjust_joint('joint3', -self.step_medium)
        elif key in 'S':
            self.adjust_joint('joint3', +self.step_medium)
        
        # Joint 4 - Wrist
        elif key in 'z':
            self.adjust_joint('joint4', -self.step_fine)
        elif key in 'x':
            self.adjust_joint('joint4', +self.step_fine)
        elif key in 'Z':
            self.adjust_joint('joint4', -self.step_coarse)
        elif key in 'X':
            self.adjust_joint('joint4', +self.step_coarse)
        
        # Gripper
        elif key == 'g':
            self.pub.gripper_open()
            print("\n[OPENED]", end=' ')
        elif key == 'h':
            self.pub.gripper_close()
            print("\n[CLOSED]", end=' ')
        
        # Utilities
        elif key == 'r':
            self.move_to_pose({'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0})
            print("\n[RESET]", end=' ')
        
        elif key == 'p':
            print(f"\n\nPose: {self.target_pose}")
            if self.sub.have_joint_states:
                print(f"Actual: {self.sub.joint_positions}\n")
        
        elif key == 'P':
            print(f"\n\n{'='*70}")
            print("COPY THIS INTO arm1.py:")
            print(f"    joint2 = {self.target_pose['joint2']:.3f}")
            print(f"    joint3 = {self.target_pose['joint3']:.3f}")
            print(f"    joint4 = {self.target_pose['joint4']:.3f}")
            print("="*70 + "\n")
        
        elif key == '\x1b':  # ESC
            return False
        
        return True


def main(args=None):
    rclpy.init(args=args)
    calibrator = ArmCalibrator()
    
    print("\n" + "="*70)
    print("Waiting for ROS2 connections...")
    print("="*70)
    
    # Spin a few times to establish connections
    for _ in range(5):
        rclpy.spin_once(calibrator, timeout_sec=0.2)
    
    print("\n✓ Ready! Press keys to move arm (no Enter needed)\n")
    print("Try: r (reset), then w/s to move\n")
    
    # Set terminal to raw mode for immediate key detection
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(fd)
        
        while True:
            # Spin ROS once
            rclpy.spin_once(calibrator, timeout_sec=0.001)
            
            # Check for immediate keypress (non-blocking)
            import select
            if select.select([sys.stdin], [], [], 0.001)[0]:
                key = sys.stdin.read(1)
                if not calibrator.handle_key(key):
                    break
    
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\n\n✓ Calibration complete!\n")
    
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
