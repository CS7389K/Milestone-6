from sensor_msgs.msg import JointState

from .constants import JOINT_NAMES


class TeleopSubscriber:
    """
    Subscriber for robot joint states.

    Monitors joint positions and maintains current state information
    for arm control operations.
    """

    def __init__(self, node, callback=None):
        """
        Initialize the joint state subscriber.

        Args:
            node: The parent ROS2 Node instance
            callback: Optional callback function to invoke with joint state updates
        """
        self._node = node
        self._callback = callback

        # State tracking
        self._have_js = False
        self._js_pos = {j: 0.0 for j in JOINT_NAMES}
        self._target_pos = {j: 0.0 for j in JOINT_NAMES}
        self.home = None

        # Create subscription
        self._subscription = node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10
        )

    def _on_joint_state(self, msg: JointState):
        """Callback for joint state updates."""
        # Map name->index for positions
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        updated = False

        for j in JOINT_NAMES:
            if j in name_to_idx and name_to_idx[j] < len(msg.position):
                self._js_pos[j] = float(msg.position[name_to_idx[j]])
                updated = True

        if updated and not self._have_js:
            # Initialize targets to current positions on first reception
            self.home = dict(self._js_pos)
            self._target_pos = dict(self._js_pos)
            self._have_js = True
            self._node.get_logger().debug('Joint states received; targets initialized.')

        # Invoke user callback if provided
        if self._callback and updated:
            self._callback(self._js_pos)

    @property
    def have_joint_states(self):
        """Check if joint states have been received."""
        return self._have_js

    @property
    def joint_positions(self):
        """Get current joint positions."""
        return dict(self._js_pos)

    @property
    def target_positions(self):
        """Get target joint positions."""
        return dict(self._target_pos)

    def update_target(self, joint_name: str, delta: float):
        """
        Update target position for a joint.

        Args:
            joint_name: Name of the joint to update
            delta: Change in position (radians)

        Returns:
            bool: True if update was successful
        """
        if joint_name not in self._target_pos:
            self._node.get_logger().warn(f'Unknown joint: {joint_name}')
            return False

        if not self._have_js:
            self._node.get_logger().warn('No /joint_states yet; cannot update target.')
            return False

        self._target_pos[joint_name] += float(delta)
        return True
