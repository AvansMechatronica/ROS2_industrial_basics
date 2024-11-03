#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "range_sensors_interfaces/msg/sensor_information.hpp"

class SensorInformationSubscriber : public rclcpp::Node {
public:
    SensorInformationSubscriber() : Node("sensor_information_subscriber") {
        // Create a subscription to the 'sensor_info' topic
        subscription_ = this->create_subscription<range_sensors_interfaces::msg::SensorInformation>(
            "sensor_info",
            10,
            std::bind(&SensorInformationSubscriber::sensor_info_callback, this, std::placeholders::_1));
    }

private:
    void sensor_info_callback(const range_sensors_interfaces::msg::SensorInformation::SharedPtr sensor_info) {
        // Log the received sensor information
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", sensor_info->data.c_str());
        // Adjust the logging statement based on the actual data structure of SensorInformation
    }

    rclcpp::Subscription<range_sensors_interfaces::msg::SensorInformation>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorInformationSubscriber>());
    rclcpp::shutdown();
    return 0;
}
