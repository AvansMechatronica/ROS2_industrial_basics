import rclpy
from rclpy.node import Node

from range_sensors_interfaces.msg import SensorInformation


class SensorInformationSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_information_subscriber')
        self.subscription = self.create_subscription(
            SensorInformation,
            'sensor_info',
            self.sensor_info_callback,
            10)
        self.subscription  # prevent unused variable warning

    def sensor_info_callback(self, sensor_info):
        self.get_logger().info('I heard: "%s"' % sensor_info)


def main(args=None):
    rclpy.init(args=args)

    sensor_information_subscriber = SensorInformationSubscriber()

    rclpy.spin(sensor_information_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_information_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()