import yaml
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from ament_index_python.packages import get_package_share_directory
import os 
from bipedal_robot_pkg.ros_nodes.logger.logging_factory import LoggingFactory

class MovementControllerNode(Node):
    """
    Central controller responsible for executing locomotion instructions.

    This node subscribes to high-level commands such as "idle_status"
    and "walk_forward", and is responsible for translating them into
    low-level actuator instructions for both legs.
    It manages motion patterns defined in YAML files, executing them
    in a time-synchronized manner.
    """
    def __init__(self):
        super().__init__("movement_controller")
        try:
            self._init_publishers()
            self._init_subscribers()
            
            # Load the patterns index from the package share directory
            pkg_share = get_package_share_directory("bipedal_robot_pkg")
            self._patterns_path = os.path.join(pkg_share, "behaviour_tree/patterns/all_patterns.yaml")

            # Load the pattern index which maps names to individual YAML files
            with open(self._patterns_path, 'r') as file:
                loaded_patterns = yaml.safe_load(file)

            # Convert relative paths to absolute paths
            base_dir = os.path.dirname(self._patterns_path)
            self._patterns = {}
            for name, rel_path in loaded_patterns.items():
                abs_path = os.path.join(base_dir, rel_path)
                self._patterns[name] = abs_path

            self._current_pattern = None                  # Currently active motion sequence
            self._pattern_start_time = None               # Reference time for pattern playback
            self._pattern_step_index = 0                  # Current index in step sequence
            self._active_pattern_name = None              # NEW: holds currently running pattern name
            
            self._timer = self.create_timer(0.05, self._pattern_timer_callback)  # ~20 Hz control loop

            self.get_logger().info("Movement Controller Node has been started.")
            self._json_logger = LoggingFactory("movement_controller_logger")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MovementControllerNode: {e}")

    def _init_publishers(self):
        self._left_leg_publisher = self.create_publisher(
            Float32MultiArray,
            "left_leg_instruction",
            10
        )
        self._right_leg_publisher = self.create_publisher(
            Float32MultiArray,
            "right_leg_instruction",
            10
        )
    
    def _init_subscribers(self):
        self._idle_publisher = self.create_subscription(
            String,
            "idle_status",
            self._idle_callback,
            10
        )
        self._walk_forward_publisher = self.create_subscription(
            String,
            "walk_forward",
            self._walk_forward_callback,
            10
        )

    def _idle_callback(self, instruction):
        """
        Callback triggered when the IdleBehaviour node becomes active.
        Loads and begins execution of the 'idle_pattern' motion pattern.
        """
        self._load_and_start_pattern("idle")

    def _walk_forward_callback(self, instruction):
        """
        Callback triggered when the WalkForwardBehaviour node becomes active.
        Loads and begins execution of the 'walk_forward' motion pattern.
        """
        self._load_and_start_pattern("walk_forward")
    
    def _load_and_start_pattern(self, pattern_name):
        """
        Loads a motion pattern from YAML and prepares it for timed execution.

        Args:
            pattern_name (str): Name of the pattern to load, as defined in patterns.yaml
        """

        # ----------------------------
        # NEW: Prevent constant reload
        # ----------------------------
        if self._active_pattern_name == pattern_name:
            return

        try:
            path = self._patterns.get(pattern_name)
            if path is None:
                self.get_logger().error(f"Pattern '{pattern_name}' not found in index.")
                self._json_logger.log("ERROR", "Pattern Not Found", {"pattern": pattern_name})
                return
            
            with open(path, 'r') as f:
                loaded_pattern = yaml.safe_load(f)
                if pattern_name not in loaded_pattern:
                    raise ValueError(f"No steps found for pattern '{pattern_name}'")
                self._current_pattern = loaded_pattern[pattern_name]

            self._pattern_start_time = time.time()
            self._pattern_step_index = 0
            self._active_pattern_name = pattern_name  # NEW: mark pattern as active

            self.get_logger().info(f"Loaded pattern {pattern_name}")
            self._json_logger.log("INFO", "Pattern Loaded", {"pattern": pattern_name})
        except Exception as e:
            self.get_logger().error(f"Failed to load/start pattern '{pattern_name}': {e}")
            self._json_logger.log("ERROR", "Pattern Load Failed", {"pattern": pattern_name, "error": str(e)})

    def _pattern_timer_callback(self):
        """
        Timer-driven loop for executing motion pattern steps in real time.

        Steps are executed when their defined timestamp (relative to pattern start)
        is reached or surpassed. When all steps have been processed, the current
        pattern is cleared.
        """
        try:
            if self._current_pattern is None or self._pattern_start_time is None:
                return

            elapsed = time.time() - self._pattern_start_time

            # Iterate through all due steps in current pattern
            while (self._pattern_step_index < len(self._current_pattern) and
                   self._current_pattern[self._pattern_step_index]["time"] <= elapsed):
                step = self._current_pattern[self._pattern_step_index]
                self._execute_joints(step.get("joints", {}))
                self._pattern_step_index += 1

            if self._pattern_step_index >= len(self._current_pattern):
                self._current_pattern = None
                self._active_pattern_name = None  # NEW: allow next pattern to start
                self.get_logger().info("Pattern execution completed.")
                self._json_logger.log("INFO", "Pattern Execution Completed", {})
        except Exception as e:
            self.get_logger().error(f"Pattern timer callback failed: {e}")

    def _execute_joints(self, joints):
        """
        Sends joint commands to the robot legs based on current step.

        Args:
            joints (dict): Dictionary with joint names as keys and angles in degrees as values.
        """
        try:
            left_msg = Float32MultiArray()
            right_msg = Float32MultiArray()

            left_msg.data = [
                float(joints.get("knee_left", 0.0)),
                float(joints.get("hip_left", 0.0)),
            ]
            right_msg.data = [
                float(joints.get("knee_right", 0.0)),
                float(joints.get("hip_right", 0.0)),
            ]

            self._left_leg_publisher.publish(left_msg)
            self._right_leg_publisher.publish(right_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to execute joint command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MovementControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
