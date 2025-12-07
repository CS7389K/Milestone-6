#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Import local modules for YOLO monitoring and teleop control
from .yolo.subscriber import YOLOSubscriber
from .yolo.yolo_data import YOLOData
from .teleop.publisher import TeleopPublisher
from .teleop.subscriber import TeleopSubscriber


def get_user_input():
    """
    Helper function to get user input.
    Returns None on EOF (Ctrl+D).
    """
    try:
        return input()
    except EOFError:
        return None


class M6Client(Node):
    """
    Milestone 6 Client for remote robot monitoring and control.
    
    This client provides remote access to:
    - YOLO object detection monitoring (subscribe to detections from server)
    - Manual teleop control (override autonomous behavior)
    
    Usage:
        # Basic monitoring (no manual control)
        client = M6Client(use_teleop=False)
        detection = client.get_latest_detection()
        
        # With manual control enabled
        client = M6Client(use_teleop=True)
        client.manual_control(linear_x=0.1, angular_z=0.0)
        client.stop_robot()
    """
    
    def __init__(self, use_yolo=True, use_teleop=False):
        """
        Initialize the M6 Client.
        
        Args:
            use_yolo (bool): Enable YOLO detection monitoring
            use_teleop (bool): Enable manual teleop control (overrides autonomous)
        """
        super().__init__('m6_client')
        
        # Store feature flags
        self.use_yolo = use_yolo
        self.use_teleop = use_teleop
        
        # YOLO detection tracking
        self.latest_detection = None
        
        # Initialize YOLO subscriber to monitor detections from server
        if self.use_yolo:
            self.yolo_subscriber = YOLOSubscriber(self, callback=self._yolo_callback)
            self.get_logger().info("YOLO subscriber initialized - monitoring detections")
        
        # Initialize teleop publisher for manual control
        if self.use_teleop:
            self.teleop_subscriber = TeleopSubscriber(self)
            self.teleop = TeleopPublisher(self, teleop_subscriber=self.teleop_subscriber)
            self.get_logger().info("Teleop control initialized - manual control enabled")
            self.get_logger().warn("Manual teleop will override autonomous behavior!")
        
        self.get_logger().info("M6 Client node initialized")

    def _yolo_callback(self, data: YOLOData):
        """
        Callback for receiving YOLO detections from the server.
        
        Args:
            data: YOLOData object containing detection information
        """
        self.latest_detection = data
        self.get_logger().info(
            f"[YOLO] Class: {data.clz}, "
            f"BBox: ({data.bbox_x:.1f}, {data.bbox_y:.1f}, "
            f"{data.bbox_w:.1f}x{data.bbox_h:.1f})"
        )

    def get_latest_detection(self):
        """
        Get the most recent YOLO detection.
        
        Returns:
            YOLOData object or None if no detection received yet
        """
        return self.latest_detection
    
    def manual_control(self, linear_x: float = 0.0, angular_z: float = 0.0):
        """
        Send manual velocity command to robot.
        
        Args:
            linear_x: Linear velocity in x direction (m/s)
            angular_z: Angular velocity around z axis (rad/s)
        """
        if not self.use_teleop:
            self.get_logger().warn("Teleop not enabled. Initialize with use_teleop=True")
            return
        
        self.teleop.set_velocity(linear_x=linear_x, angular_z=angular_z)
        self.get_logger().info(f"Manual control: linear={linear_x:.3f}, angular={angular_z:.3f}")

    def stop_robot(self):
        """Stop the robot (set all velocities to zero)."""
        if self.use_teleop:
            self.teleop.stop()
            self.get_logger().info("Robot stopped")
        else:
            self.get_logger().warn("Teleop not enabled")


def main(args=None):
    """
    Main entry point for the M6 Client.
    
    Remote monitoring and control interface for the robot.
    
    Features:
    - Real-time YOLO detection monitoring
    - Manual teleop control (optional)
    
    Commands:
    - 'status': Show latest YOLO detection
    - 'stop': Stop robot (if teleop enabled)
    - 'forward': Move forward (if teleop enabled)
    - 'backward': Move backward (if teleop enabled)
    - 'left': Turn left (if teleop enabled)
    - 'right': Turn right (if teleop enabled)
    - 'faster': Increase speed (if teleop enabled)
    - 'slower': Decrease speed (if teleop enabled)
    - 'quit' or Ctrl+D: Exit
    """
    rclpy.init(args=args)

    print("\n" + "="*60)
    print("   Milestone 6 Client - Remote Robot Monitor & Control")
    print("="*60)
    print("\nInitializing M6 Client Node...")
    print("Note: Set use_teleop=True in code to enable manual control")
    
    # Initialize client - change use_teleop=True to enable manual control
    client = M6Client(use_yolo=True, use_teleop=False)
    
    print("M6 Client Node Initialized.\n")
    print("Available commands:")
    print("  'status'   - Show latest YOLO detection")
    if client.use_teleop:
        print("  'forward'  - Move robot forward")
        print("  'backward' - Move robot backward")
        print("  'left'     - Turn left")
        print("  'right'    - Turn right")
        print("  'faster'   - Increase speed")
        print("  'slower'   - Decrease speed")
        print("  'stop'     - Stop all movement")
    print("  'quit'     - Exit client")
    print("="*60 + "\n")

    # Speed control for teleop
    linear_speed = 0.15
    angular_speed = 0.5
    speed_increment = 0.05
    
    try:
        while True:
            print("Command: ", end='', flush=True)
            user_input = get_user_input()
            
            if user_input is None or user_input.lower() == 'quit':
                print("\nExiting...")
                break
            
            command = user_input.lower().strip()
            
            # Handle status command
            if command == 'status':
                detection = client.get_latest_detection()
                if detection:
                    print(f"\n[DETECTION] Class: {detection.clz}")
                    print(f"  Position: ({detection.bbox_x:.1f}, {detection.bbox_y:.1f})")
                    print(f"  Size: {detection.bbox_w:.1f} x {detection.bbox_h:.1f} pixels\n")
                else:
                    print("\n[NO DETECTION] Waiting for detections from server...\n")
                continue
            
            # Handle teleop commands
            if client.use_teleop:
                if command == 'stop':
                    client.stop_robot()
                    print(f"[TELEOP] Robot stopped\n")
                    
                elif command == 'forward':
                    client.manual_control(linear_x=linear_speed, angular_z=0.0)
                    print(f"[TELEOP] Moving forward at {linear_speed:.2f} m/s\n")
                    
                elif command == 'backward':
                    client.manual_control(linear_x=-linear_speed, angular_z=0.0)
                    print(f"[TELEOP] Moving backward at {linear_speed:.2f} m/s\n")
                    
                elif command == 'left':
                    client.manual_control(linear_x=0.0, angular_z=angular_speed)
                    print(f"[TELEOP] Turning left at {angular_speed:.2f} rad/s\n")
                    
                elif command == 'right':
                    client.manual_control(linear_x=0.0, angular_z=-angular_speed)
                    print(f"[TELEOP] Turning right at {angular_speed:.2f} rad/s\n")
                    
                elif command == 'faster':
                    linear_speed = min(linear_speed + speed_increment, 0.5)
                    angular_speed = min(angular_speed + speed_increment, 2.0)
                    print(f"[TELEOP] Speed increased: linear={linear_speed:.2f}, angular={angular_speed:.2f}\n")
                    
                elif command == 'slower':
                    linear_speed = max(linear_speed - speed_increment, 0.05)
                    angular_speed = max(angular_speed - speed_increment, 0.1)
                    print(f"[TELEOP] Speed decreased: linear={linear_speed:.2f}, angular={angular_speed:.2f}\n")
                    
                elif command:
                    print(f"[ERROR] Unknown command: '{command}'\n")
            else:
                if command in ['stop', 'forward', 'backward', 'left', 'right', 'faster', 'slower']:
                    print("[ERROR] Teleop not enabled. Reinitialize with use_teleop=True\n")
                elif command:
                    print(f"[ERROR] Unknown command: '{command}'\n")
                    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user...")
    finally:
        print("\nShutting down client...")
        if client.use_teleop:
            client.stop_robot()
        client.destroy_node()
        rclpy.shutdown()
        print("Client shutdown complete.\n")


if __name__ == '__main__':
    main()