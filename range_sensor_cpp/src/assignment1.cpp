#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "range_sensors_interfaces/msg/sensor_information.hpp"
#include "range_sensors_interfaces/msg/box_height_information.hpp"

const double SENSOR_CONVEYOR_BELT_DISTANCE = 1.0; // Metres: height above the conveyor belt
const double SENSOR_USABLE_RANGE = 0.9; // Metres: usable sensor range according to the datasheet

class BoxHeightCalculator : public rclcpp::Node {
public:
    BoxHeightCalculator() : Node("box_height_calculator") {
        box_height_publisher_ = this->create_publisher<range_sensors_interfaces::msg::BoxHeightInformation>(
            "box_height_info", 10);

        sensor_info_subscription_ = this->create_subscription<range_sensors_interfaces::msg::SensorInformation>(
            "sensor_info",
            10,
            std::bind(&BoxHeightCalculator::sensor_info_callback, this, std::placeholders::_1));
    }

private:
    void sensor_info_callback(const range_sensors_interfaces::msg::SensorInformation::SharedPtr sensor_info) {
        double box_distance = sensor_info->sensor_data.range;

        // Compute the height of the box.
        // Boxes that are detected to be shorter than 10cm are due to sensor noise.
        // Do not publish information about them.
        if (box_distance <= SENSOR_USABLE_RANGE) {
            // Declare a message object for publishing the box height information.
            range_sensors_interfaces::msg::BoxHeightInformation box_height_info;
            // Update height of box.
            box_height_info.box_height = SENSOR_CONVEYOR_BELT_DISTANCE - box_distance;
            
            // Only publish if the box height is more than 10 cm
            if (box_height_info.box_height > 0.1) { // 0.1 m = 10 cm
                box_height_publisher_->publish(box_height_info);
                RCLCPP_INFO(this->get_logger(), "Published box height: %.2f m", box_height_info.box_height);
            }
        }
    }

    // Publisher and subscription for the node
    rclcpp::Publisher<range_sensors_interfaces::msg::BoxHeightInformation>::SharedPtr box_height_publisher_;
    rclcpp::Subscription<range_sensors_interfaces::msg::SensorInformation>::SharedPtr sensor_info_subscription_;
};

int main(int argc, char * argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the BoxHeightCalculator
    auto box_height_calculator = std::make_shared<BoxHeightCalculator>();

    // Spin the node to keep it active
    rclcpp::spin(box_height_calculator);

    // Clean up and shut down
    rclcpp::shutdown();
    return 0;
}
