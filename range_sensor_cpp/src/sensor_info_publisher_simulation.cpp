#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "range_sensors_interfaces/msg/sensor_information.hpp"

using namespace std::chrono_literals;

// Constants
const float MAX_RANGE = 1.00;
const float MIN_RANGE = 0.10;

class SensorInformationPublisher : public rclcpp::Node {
public:
    SensorInformationPublisher() : Node("sensor_information_publisher") {
        // Initialize publisher
        publisher_ = this->create_publisher<range_sensors_interfaces::msg::SensorInformation>("sensor_info", 10);

        // Initialize timer with a callback interval of 0.5 seconds
        timer_ = this->create_wall_timer(500ms, std::bind(&SensorInformationPublisher::timer_callback, this));

        // Set up static sensor information
        sensor_info_.maker_name = "Avans";
        sensor_info_.part_number = 20241101;
        sensor_info_.sensor_data.header.frame_id = "distance_sensor_frame";
        sensor_info_.sensor_data.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        sensor_info_.sensor_data.field_of_view = 0.5;
        sensor_info_.sensor_data.min_range = MIN_RANGE;
        sensor_info_.sensor_data.max_range = MAX_RANGE;
    }

private:
    void timer_callback() {
        // Set the timestamp in the header
        sensor_info_.sensor_data.header.stamp = this->get_clock()->now();

        // Simulate sensor data
        if (distribution_(random_engine_) < 0.95) {
            sensor_info_.sensor_data.range = MAX_RANGE;
        } else {
            std::uniform_real_distribution<float> range_distribution(MIN_RANGE, MAX_RANGE);
            sensor_info_.sensor_data.range = range_distribution(random_engine_);
        }

        // Publish the sensor information message
        publisher_->publish(sensor_info_);
    }

    // Publisher and timer
    rclcpp::Publisher<range_sensors_interfaces::msg::SensorInformation>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Sensor information message
    range_sensors_interfaces::msg::SensorInformation sensor_info_;

    // Random number generator for simulating data
    std::default_random_engine random_engine_;
    std::uniform_real_distribution<float> distribution_{0.0, 1.0};
};

int main(int argc, char * argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the sensor information publisher node
    auto node = std::make_shared<SensorInformationPublisher>();
    rclcpp::spin(node);

    // Clean up and shut down
    rclcpp::shutdown();
    return 0;
}
