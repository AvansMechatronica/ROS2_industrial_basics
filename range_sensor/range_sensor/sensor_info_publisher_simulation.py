import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from sensor_msgs.msg import Range
from range_sensors_interfaces.msg import SensorInformation
import random

MAX_RANGE = 1.00
MIN_RANGE = 0.10

class SensorInformationPublisher(Node):
    def __init__(self):
        super().__init__('sensor_information_publisher')
        self.publisher_ = self.create_publisher(SensorInformation, 'sensor_info', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Static values of the sensor_information
        self.sensor_info = SensorInformation()
        self.sensor_info.maker_name = "Avans"
        self.sensor_info.part_number = 20241101
        self.sensor_info.sensor_data.header.frame_id = 'distance_sensor_frame'
        self.sensor_info.sensor_data.radiation_type = Range.ULTRASOUND
        self.sensor_info.sensor_data.field_of_view = 0.5 # Field of view of the sensor in rad.
        self.sensor_info.sensor_data.min_range = MIN_RANGE # Minimum distance range of the sensor in m.
        self.sensor_info.sensor_data.max_range = MAX_RANGE # Maximum distance range of the sensor in m.


    def timer_callback(self):
            # Fill in the header information.
        self.sensor_info.sensor_data.header.stamp = self.get_clock().now().to_msg()#rclpy.time.Time()
 
        # Fill in the simulated sensor data information.
        if(random.uniform(0.0, 1.0) < 0.95):
            self.sensor_info.sensor_data.range =  MAX_RANGE
        else:
            self.sensor_info.sensor_data.range = random.uniform(MIN_RANGE, MAX_RANGE)

        self.publisher_.publish(self.sensor_info)
        #self.get_logger().info('Publishing: "%s"' % self.sensor_info)


def main(args=None):
    rclpy.init(args=args)

    sensor_information_publisher = SensorInformationPublisher()

    rclpy.spin(sensor_information_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_information_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()