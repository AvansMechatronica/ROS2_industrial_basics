#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "range_sensors_interfaces/msg/box_height_information.hpp"
#include "range_sensors_interfaces/srv/convert_metres_to_inches.hpp"

class Assignment2 : public rclcpp::Node {
public:
    Assignment2() : Node("assignment2_async") {
        // Create the client for the service to convert metres to inches
        convert_meters_to_inches_client_ = this->create_client<range_sensors_interfaces::srv::ConvertMetresToInches>("metres_to_inches");
        
        // Wait for the service to be available
        while (!convert_meters_to_inches_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        // Subscription to the box height information topic
        subscription_ = this->create_subscription<range_sensors_interfaces::msg::BoxHeightInformation>(
            "box_height_info", 10, std::bind(&Assignment2::box_height_callback, this, std::placeholders::_1));
    }

private:
    void send_request(double metres) {
        // Create a request for the service
        auto request = std::make_shared<range_sensors_interfaces::srv::ConvertMetresToInches::Request>();
        request->distance_metres = metres;

        // Call the service asynchronously
        auto response_future = convert_meters_to_inches_client_->async_send_request(request);
        
        // Handle the response once it's ready
        std::move(response_future).then(
            [this, metres](rclcpp::Future<range_sensors_interfaces::srv::ConvertMetresToInches::Response>::SharedPtr future_response) {
                try {
                    auto response = future_response->get();
                    if (response.success) {
                        RCLCPP_INFO(this->get_logger(), "The height of the box is %.2f metres or %.2f inches", metres, response.distance_inches);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Invalid request value");
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }

    void box_height_callback(const range_sensors_interfaces::msg::BoxHeightInformation::SharedPtr box_height) {
        send_request(box_height->box_height);
    }

    rclcpp::Client<range_sensors_interfaces::srv::ConvertMetresToInches>::SharedPtr convert_meters_to_inches_client_;
    rclcpp::Subscription<range_sensors_interfaces::msg::BoxHeightInformation>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the Assignment2 node
    auto assignment2 = std::make_shared<Assignment2>();

    // Spin the node to keep it active
    rclcpp::spin(assignment2);

    // Clean up and shut down
    rclcpp::shutdown();
    return 0;
}
