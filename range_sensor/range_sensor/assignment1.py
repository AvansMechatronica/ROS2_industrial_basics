# Assignment 1 for Week1: In this assignment you will subscribe to the topic that
# publishes sensor information. Then, you will transform the sensor reading from
# the reference frame of the sensor to compute the height of a box based on the
# illustration shown in the assignment document. Then, you will publish the box height
# on a new message type ONLY if the height of the box is more than 10cm.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Range
from range_sensors_interfaces.msg import SensorInformation, BoxHeightInformation

_SENSOR_CONVEYOR_BELT_DISTANCE = 1.0 # Metres -> assume that the sensor is placed 1.0 above the conveyor belt
_SENSOR_USABLE_RANGE = 0.9 # Metres -> Usable sensor range, accordingly datasheet


class BoxHeightCalculator(Node):

    def __init__(self):
        super().__init__('box_height_calculator')

        self.box_height_publisher = self.create_publisher(BoxHeightInformation, 'box_height_info', 10)

        self.sensor_info_subscription = self.create_subscription(
            SensorInformation,
            'sensor_info',
            self.sensor_info_callback,
            10)
        self.sensor_info_subscription  # prevent unused variable warning

    def sensor_info_callback(self, sensor_info):
        #self.get_logger().info('I heard: "%s"' % sensor_info)
        box_distance = sensor_info.sensor_data.range

        # Compute the height of the box.
        # Boxes that are detected to be shorter than 10cm are due to sensor noise.
        # Do not publish information about them.
        if box_distance > _SENSOR_USABLE_RANGE:
            pass
        else:
            # Declare a message object for publishing the box height information.
            box_height_info = BoxHeightInformation()
            # Update height of box.
            box_height_info.box_height = _SENSOR_CONVEYOR_BELT_DISTANCE - box_distance
            # Publish box height using the publisher argument passed to the callback function.
            self.box_height_publisher.publish(box_height_info)
 

def main(args=None):
    rclpy.init(args=args)
    

    box_height_calculator = BoxHeightCalculator()

    rclpy.spin(box_height_calculator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    box_height_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()