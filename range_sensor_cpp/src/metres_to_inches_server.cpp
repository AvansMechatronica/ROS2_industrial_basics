#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "range_sensors_interfaces/srv/convert_metres_to_inches.hpp"

class MetresToInchesService : public rclcpp::Node {
public:
    using ConvertMetresToInches = range_sensors_interfaces::srv::ConvertMetresToInches;

    MetresToInchesService() : Node("metres_to_inches_service") {
        service_ = this->create_service<ConvertMetresToInches>("metres_to_inches",
            std::bind(&MetresToInchesService::metres_to_inches_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void metres_to_inches_callback(const std::shared_ptr<ConvertMetresToInches::Request> request,
                                    std::shared_ptr<ConvertMetresToInches::Response> response) {
        if (request->distance_metres < 0) {
            response->success = false;
            response->distance_inches = -std::numeric_limits<double>::infinity(); // Default error value.
        } else {
            response->distance_inches = request->distance_metres * METERS_TO_INCHES;
            response->success = true;
        }
        // Logging for incoming requests and outgoing responses (uncomment for debugging)
        // RCLCPP_INFO(this->get_logger(), "Incoming request: distance_metres: %f", request->distance_metres);
        // RCLCPP_INFO(this->get_logger(), "Outgoing response: success: %s, distance_inches: %f", response->success ? "true" : "false", response->distance_inches);
    }

    rclcpp::Service<ConvertMetresToInches>::SharedPtr service_;
    static constexpr double METERS_TO_INCHES = 39.3700787;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetresToInchesService>());
    rclcpp::shutdown();
    return 0;
}
