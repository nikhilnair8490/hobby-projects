# Import rclpy dependencies
import rclpy
from rclpy.node import Node

# Import Raspberry Pi Sensor module
import led_control.led_raspi as raspi

# Import messages
from sensor_msgs.msg import Range

# Create class and inherit from parent class 'Node'
class LedControl(Node):
    """LED Control"""

    def __init__(self):
        """Constructor for the class"""

        # Inherit parent class and add node name as arg
        super().__init__("led_control_node")

        # Create Subscriber
        self.rangeSubscriber = self.create_subscription(
            Range, "distance", self.subsCallbackFunc, 10
        )

        self.rangeSubscriber  # prevent unused variable warning

    def subsCallbackFunc(self, msg):
        """Callback function for subscriber"""

        currDist = (
            msg.range
        )  # range data from distance topic published by ultrasonic sensor

        # LED ON if distance is less than 5 cm and off if greater than 8 cm
        if currDist > 0.08:
            raspi.LED_OFF()
        elif currDist < 0.05:
            raspi.LED_ON()
        else:
            # Continue last state of LED #
            pass


def main(args=None):
    # Initialise context and ROS communications
    rclpy.init(args=args)

    # Instantiate a node from the class
    led_control_node = LedControl()

    # Spin the node, pass if CTRL+C interrupt
    try:
        rclpy.spin(led_control_node)
    except KeyboardInterrupt:
        pass

    # Run GPIO clean up on Raspberry Pi
    raspi.GPIOCleanup()

    # Destroy the node once it stops executing to free resources
    led_control_node.destroy_node()

    # Shutdown the intialised context
    rclpy.shutdown()


# It is used to execute main() only if the file was run as a script directly, and not imported
# as a module
if __name__ == "__main__":
    main()
