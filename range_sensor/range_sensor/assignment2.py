#! /usr/bin/env python

# Assignment 2 for Week1: In this assignment you will subscribe to the topic that
# publishes information on the box height in metres and use the metres_to_feet
# service to convert this height in metres to height in feet.


import rclpy
from rclpy.node import Node
from range_sensors_interfaces.msg import BoxHeightInformation
from range_sensors_interfaces.srv import ConvertMetresToInches

class Assignment2(Node):

    def __init__(self):
        super().__init__('assignment2_async')
        self.convert_meters_to_inches_client = self.create_client(ConvertMetresToInches, 'metres_to_inches')
        while not self.convert_meters_to_inches_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = ConvertMetresToInches.Request()

        self.subscription = self.create_subscription(
            BoxHeightInformation,
            'box_height_info',
            self.box_height_callback,
            10)
        self.subscription  # prevent unused variable warning

    def send_request(self, metres):
        self.request.distance_metres = metres
        response = self.convert_meters_to_inches_client.call_async(self.request)
        if  response.success:
            self.get_logger().info('I heard: "%s"' % response.Response)
            #self.get_logger().info("The hight of the box is %4.2f metres or %4.2f inches" %(metres, response.distance_inches)) 
        else:
            metres_to_inches_client.get_logger().info("Invalis request value")



        # Write a log message here to print the height of this box in feet.

    def box_height_callback(self, box_height):
        self.send_request(box_height.box_height)


def main():
    rclpy.init()

    assignment2 = Assignment2()

    rclpy.spin(assignment2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    assignment2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
