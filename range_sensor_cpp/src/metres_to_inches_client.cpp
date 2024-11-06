#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "range_sensors_interfaces/srv/convert_metres_to_inches.hpp"

class MetresToInchesClientAsync : public rclcpp::Node {
public:
    using ConvertMetresToInches = range_sensors_interfaces::srv::ConvertMetresToInches;
    using Client = rclcpp::Client<ConvertMetresToInches>;

    MetresToInchesClientAsync() : Node("metres_to_inches_client_async") {
        client_ = this->create_client<ConvertMetresToInches>("metres_to_inches");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

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

private:
    rclcpp::Client<ConvertMetresToInches>::SharedPtr client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    if (argc != 2) {
        std::cerr << "Usage: metres_to_inches_client_async <distance_in_metres>" << std::endl;
        return 1;
    }

    double dist_metres = std::stod(argv[1]);
    auto metres_to_inches_client = std::make_shared<MetresToInchesClientAsync>();
    metres_to_inches_client->send_request(dist_metres);

    rclcpp::shutdown();
    return 0;
}
