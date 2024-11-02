from range_sensors_interfaces.srv import ConvertMetresToInches

import rclpy
from rclpy.node import Node
import numpy as np

METERS_TO_INCHES = 39.3700787

class MetresToInchesService(Node):

    def __init__(self):
        super().__init__('metres_to_inch_service')
        self.metrest_to_inches_service = self.create_service(ConvertMetresToInches, 'metres_to_inches', self.metres_to_inches_callback)

    def metres_to_inches_callback(self, request, response):
        if(request.distance_metres < 0):
            response.success = False
            response.distance_inches = -np.Inf # Default error value.
        else:
            response.distance_inches = request.distance_metres * METERS_TO_INCHES
            response.success = True
        #self.get_logger().info('Incoming request\na: "%s"' % request)
        #self.get_logger().info('Outgoing response\na: "%s"' % response)

        return response


def main():
    rclpy.init()
    metres_to_inches_service = MetresToInchesService()
    rclpy.spin(metres_to_inches_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()