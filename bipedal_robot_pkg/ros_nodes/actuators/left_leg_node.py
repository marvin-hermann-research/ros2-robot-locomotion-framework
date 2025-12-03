from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

class LeftLegNode(Node):
    """
    Subscriber node representing the robot's left leg controller.

    This node listens for actuation commands on the "left_leg_instruction"
    topic and will later forward them to the hardware or simulation layer.
    """
    def __init__(self):
        super().__init__("left_leg")
        self._subscriber = self.create_subscription(
            Float32MultiArray,
            "left_leg_instruction",
            self._listener_callback,
            10
        )
        self.get_logger().info("Left leg subscriber initialized.")
        self._publisher = self.create_publisher(
            Int32MultiArray,
            "/left/angles",
            10
        )
        self.get_logger().info("Left leg publisher initialized.")

    def _listener_callback(self, instruction: Float32MultiArray):
        """
        Callback that handles incoming left leg instructions.
        Processes the Float32MultiArray command and forwards it to the actuator.
        """
        try:
            angles = Int32MultiArray()
            angles.data = [int(x) for x in instruction.data]
            self._publisher.publish(angles)
            self.get_logger().info(f"Published left leg angles: {angles.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing left leg instruction: {e}")
