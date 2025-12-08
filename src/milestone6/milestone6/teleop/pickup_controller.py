"""
Pickup and putdown controller for autonomous object manipulation.

Manages state machine for picking up and putting down detected objects
using the robot's gripper arm.
"""


class PickupController:
    """
    Controls the pickup and putdown sequence for detected objects.
    
    States:
    - idle: Waiting for target
    - approaching: Moving toward object
    - extending: Opening gripper and extending arm
    - grasping: Closing gripper to grasp object
    - retracting: Returning arm to home position
    - holding: Holding object, waiting for putdown command
    - extending_putdown: Extending arm to putdown location
    - releasing: Opening gripper to release object
    - retracting_putdown: Retracting arm after release
    - complete: Sequence complete
    """
    
    # State constants
    STATE_IDLE = 'idle'
    STATE_APPROACHING = 'approaching'
    STATE_EXTENDING = 'extending'
    STATE_GRASPING = 'grasping'
    STATE_RETRACTING = 'retracting'
    STATE_HOLDING = 'holding'
    STATE_EXTENDING_PUTDOWN = 'extending_putdown'
    STATE_RELEASING = 'releasing'
    STATE_RETRACTING_PUTDOWN = 'retracting_putdown'
    STATE_COMPLETE = 'complete'
    
    # Timing constants (seconds)
    GRIPPER_OPEN_TIME = 0.5
    ARM_EXTEND_TIME = 2.0
    GRIPPER_CLOSE_TIME = 1.5
    ARM_RETRACT_TIME = 2.0
    GRIPPER_RELEASE_TIME = 1.0
    
    def __init__(self, node, teleop_publisher):
        """
        Initialize the pickup controller.
        
        Args:
            node: The parent ROS2 Node instance
            teleop_publisher: TeleopPublisher instance for controlling robot
        """
        self._node = node
        self._teleop = teleop_publisher
        
        self._state = self.STATE_IDLE
        self._state_start_time = None
        self._holding_object = False
    
    @property
    def state(self):
        """Get current state."""
        return self._state
    
    @property
    def is_holding_object(self):
        """Check if currently holding an object."""
        return self._holding_object
    
    @property
    def is_busy(self):
        """Check if controller is executing a sequence."""
        return self._state not in [self.STATE_IDLE, self.STATE_HOLDING, self.STATE_COMPLETE]
    
    def reset(self):
        """Reset controller to idle state."""
        self._state = self.STATE_IDLE
        self._state_start_time = None
        self._holding_object = False
        self._node.get_logger().info("Pickup controller reset to idle")
    
    def start_pickup(self):
        """Initiate pickup sequence."""
        if self._state != self.STATE_APPROACHING:
            self._node.get_logger().info("Starting pickup sequence...")
            self._state = self.STATE_EXTENDING
            self._state_start_time = self._node.get_clock().now()
            
            # Stop base movement
            self._teleop.set_velocity(linear_x=0.0, angular_z=0.0)
            
            # Open gripper
            self._node.get_logger().info("Opening gripper...")
            self._teleop.gripper_open()
    
    def start_putdown(self):
        """Initiate putdown sequence."""
        if self._state == self.STATE_HOLDING:
            self._node.get_logger().info("Starting putdown sequence...")
            self._state = self.STATE_EXTENDING_PUTDOWN
            self._state_start_time = self._node.get_clock().now()
            
            # Stop base movement
            self._teleop.set_velocity(linear_x=0.0, angular_z=0.0)
            
            # Extend arm to putdown location
            self._node.get_logger().info("Extending arm to putdown location...")
            self._teleop.move_pose('extend')
        else:
            self._node.get_logger().warn(f"Cannot putdown: not holding object (state: {self._state})")
    
    def set_approaching(self):
        """Set state to approaching (robot is moving toward object)."""
        if self._state == self.STATE_IDLE:
            self._state = self.STATE_APPROACHING
    
    def update(self):
        """
        Update the state machine. Should be called periodically.
        
        Returns:
            bool: True if state changed, False otherwise
        """
        if self._state in [self.STATE_IDLE, self.STATE_APPROACHING, self.STATE_HOLDING, self.STATE_COMPLETE]:
            return False  # No automatic transitions from these states
        
        if self._state_start_time is None:
            self._state_start_time = self._node.get_clock().now()
            return False
        
        # Calculate elapsed time in current state
        current_time = self._node.get_clock().now()
        elapsed = (current_time - self._state_start_time).nanoseconds / 1e9
        
        state_changed = False
        
        # ===== PICKUP SEQUENCE =====
        if self._state == self.STATE_EXTENDING:
            if elapsed >= self.GRIPPER_OPEN_TIME:
                # Gripper is open, now extend arm
                self._node.get_logger().info("Extending arm to object...")
                self._teleop.move_pose('extend')
                self._state = self.STATE_GRASPING
                self._state_start_time = current_time
                state_changed = True
        
        elif self._state == self.STATE_GRASPING:
            if elapsed >= self.ARM_EXTEND_TIME:
                # Arm is extended, now close gripper
                self._node.get_logger().info("Grasping object...")
                self._teleop.gripper_close()
                self._state = self.STATE_RETRACTING
                self._state_start_time = current_time
                state_changed = True
        
        elif self._state == self.STATE_RETRACTING:
            if elapsed >= self.GRIPPER_CLOSE_TIME:
                # Gripper is closed, now retract arm
                self._node.get_logger().info("Retracting arm to home position...")
                self._teleop.move_pose('home')
                self._state = self.STATE_HOLDING
                self._state_start_time = current_time
                self._holding_object = True
                state_changed = True
        
        # ===== PUTDOWN SEQUENCE =====
        elif self._state == self.STATE_EXTENDING_PUTDOWN:
            if elapsed >= self.ARM_EXTEND_TIME:
                # Arm is extended, now open gripper to release
                self._node.get_logger().info("Releasing object...")
                self._teleop.gripper_open()
                self._state = self.STATE_RELEASING
                self._state_start_time = current_time
                state_changed = True
        
        elif self._state == self.STATE_RELEASING:
            if elapsed >= self.GRIPPER_RELEASE_TIME:
                # Gripper is open, now retract arm
                self._node.get_logger().info("Retracting arm after putdown...")
                self._teleop.move_pose('home')
                self._state = self.STATE_RETRACTING_PUTDOWN
                self._state_start_time = current_time
                state_changed = True
        
        elif self._state == self.STATE_RETRACTING_PUTDOWN:
            if elapsed >= self.ARM_RETRACT_TIME:
                # Putdown complete
                self._node.get_logger().info("Putdown sequence complete!")
                self._state = self.STATE_COMPLETE
                self._state_start_time = current_time
                self._holding_object = False
                state_changed = True
        
        return state_changed
