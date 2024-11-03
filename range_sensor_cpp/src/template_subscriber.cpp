#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        // Create a subscription to the "topic"
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 
            10, 
            std::bind(&MinimalSubscriber::listener_callback, this, std::placeholders::_1)
        );
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Log the received message
        RCLCPP_INFO(this->get_logger(), "I heard: \"%s\"", msg->data.c_str());
    }

    // Subscription for the node
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the MinimalSubscriber
    auto minimal_subscriber = std::make_shared<MinimalSubscriber>();

    // Spin the node to keep it active
    rclcpp::spin(minimal_subscriber);

    // Clean up and shut down
    rclcpp::shutdown();
    return 0;
}
