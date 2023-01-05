# Import rclpy dependencies
import rclpy
from rclpy.node import Node

#Import Raspberry Pi Sensor module
import ultrasonic_sensor_interface.usensor_raspi_read as raspi

# Import messages
from sensor_msgs.msg import Range

# Create class and inherit from parent class 'Node'
class UsensorInterface(Node):
    """Ultrasonic sensor interface"""

    def __init__(self):
        """Constructor for the class UsensorInterface"""

        # Inherit parent class and add node name as arg
        super().__init__('ultrasonic_sensor')

        # Create publisher
        self.rangePublisher = self.create_publisher(Range, 'distance', 10)

        # Create timer and callback function
        self.timerCallback = self.create_timer(0.1, self.rangeCallbackFnc)

    def rangeCallbackFnc(self):
        """Callback function for publishing range"""

        # Instantiate Range object
        rangeMsg = Range()

        # Populate the fields of Range message fields
        rangeMsg.header.stamp = self.get_clock().now().to_msg()  # current system time
        rangeMsg.header.frame_id = "/sonar_link"
        rangeMsg.min_range = 0.02  # Available from HC-SR04 sensor spec
        rangeMsg.max_range = 4.0  # Available from HC-SR04 sensor spec
        rangeMsg.field_of_view = 0.261799  # Available from HC-SR04 sensor spec
        rangeMsg.radiation_type = 0  # 0 = ULTRASONIC SENSOR

        # Update the range with sensor values
        dist =  raspi.distofObj() # The output of distofObj() is the distance of object from sensor in cm
        rangeMsg.range = dist*0.01 # convert from cm to m

        # publish the range values on ROS network
        self.rangePublisher.publish(rangeMsg)


def main(args=None):
    # Initialise context and ROS communications
    rclpy.init(args=args)

    # Instantiate a node from the class
    ultrasonic_sensor = UsensorInterface()

    # Spin the node, pass if CTRL+C interrupt
    try:
        rclpy.spin(ultrasonic_sensor)
    except KeyboardInterrupt:
        pass    
    
    #Run GPIO clean up on Raspberry Pi
    raspi.GPIOCleanup()

    # Destroy the node once it stops executing to free resources
    ultrasonic_sensor.destroy_node()

    # Shutdown the intialised context
    rclpy.shutdown()


# It is used to execute main() only if the file was run as a script directly, and not imported
# as a module
if __name__ == "__main__":
    main()
