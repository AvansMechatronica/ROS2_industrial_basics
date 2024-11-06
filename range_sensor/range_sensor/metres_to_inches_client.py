import sys

from range_sensors_interfaces.srv import ConvertMetresToInches
import rclpy
from rclpy.node import Node


class MetresToInchesClientAsync(Node):

    def __init__(self):
        super().__init__('metres_to_inches_client_async')
        self.metres_to_inches_client = self.create_client(ConvertMetresToInches, 'metres_to_inches')
        while not self.metres_to_inches_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = ConvertMetresToInches.Request()

    def send_request(self, metres):
        self.request.distance_metres = metres
        return self.metres_to_inches_client.call_async(self.request)


def main():
    rclpy.init()

    metres_to_inches_client = MetresToInchesClientAsync()
    dist_metres = float(sys.argv[1])
    #dist_metres = 0.25
    future = metres_to_inches_client.send_request(dist_metres)
    rclpy.spin_until_future_complete(metres_to_inches_client, future)
    response = future.result()
    if  response.success:
        metres_to_inches_client.get_logger().info(
            'Result of metres_to_inches: for %f meters = %f inches' %
            (dist_metres, response.distance_inches))
    else:
        metres_to_inches_client.get_logger().info("Invalis request value")


    metres_to_inches_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
