#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "range_sensors_interfaces/msg/box_height_information.hpp"
#include "range_sensors_interfaces/srv/convert_metres_to_inches.hpp"

class Assignment2 : public rclcpp::Node {
public:
    using ConvertMetresToInches = range_sensors_interfaces::srv::ConvertMetresToInches;
    using BoxHeightInformation = range_sensors_interfaces::msg::BoxHeightInformation;

    Assignment2() : Node("assignment2_async") {
        
        client_ = this->create_client<ConvertMetresToInches>("metres_to_inches");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        
        // Subscription to the box height information topic
        subscription_ = this->create_subscription<range_sensors_interfaces::msg::BoxHeightInformation>(
            "box_height_info", 10, std::bind(&Assignment2::box_height_callback, this, std::placeholders::_1));
    }

private:
    void send_request(double metres) {
        auto request = std::make_shared<ConvertMetresToInches::Request>();
        request->distance_metres = metres;

        // Call the service asynchronously
        auto future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(),
                            "Result of metres_to_inches: for %f metres = %f inch",
                            metres, response->distance_inches);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid request value");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

    void box_height_callback(const range_sensors_interfaces::msg::BoxHeightInformation::SharedPtr box_height) {
        send_request(box_height->box_height);
    }
    rclcpp::Client<ConvertMetresToInches>::SharedPtr client_;
    rclcpp::Subscription<BoxHeightInformation>::SharedPtr subscription_;
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
