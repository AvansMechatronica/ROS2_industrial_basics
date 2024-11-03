#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher") {
        // Create a publisher for String messages
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        
        // Set up a timer to call the timer_callback function every 0.5 seconds
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        
        i_ = 0; // Initialize counter
    }

private:
    void timer_callback() {
        // Create and fill the message
        auto msg = std_msgs::msg::String();
        msg.data = "Welcome to the ROS2 template package (cpp-version): " + std::to_string(i_);
        
        // Publish the message
        publisher_->publish(msg);
        
        // Log the message
        RCLCPP_INFO(this->get_logger(), "Publishing: \"%s\"", msg.data.c_str());
        
        i_++; // Increment the counter
    }

    // Publisher and timer for the node
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Counter for messages
    int i_;
};

int main(int argc, char * argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the MinimalPublisher
    auto minimal_publisher = std::make_shared<MinimalPublisher>();

    // Spin the node to keep it active
    rclcpp::spin(minimal_publisher);

    // Clean up and shut down
    rclcpp::shutdown();
    return 0;
}
