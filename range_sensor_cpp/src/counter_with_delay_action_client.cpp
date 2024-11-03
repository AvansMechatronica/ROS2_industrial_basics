#include <memory>
#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "range_sensors_interfaces/action/counter_with_delay.hpp"

using namespace std::chrono_literals;

class CounterWithDelayClient : public rclcpp::Node {
public:
    using CounterWithDelay = range_sensors_interfaces::action::CounterWithDelay;
    using GoalHandleCounterWithDelay = rclcpp_action::ClientGoalHandle<CounterWithDelay>;

    CounterWithDelayClient()
    : Node("counter_with_delay_action_client") {
        this->client_ = rclcpp_action::create_client<CounterWithDelay>(this, "counter_with_delay_action_server");
    }

    void send_goal(int num_counts) {
        auto goal_msg = CounterWithDelay::Goal();
        goal_msg.num_counts = num_counts;

        // Wait for the action server to be available
        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // Send the goal
        auto send_goal_options = rclcpp_action::Client<CounterWithDelay>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CounterWithDelayClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&CounterWithDelayClient::feedback_callback, this, std::placeholders::_1);
        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void goal_response_callback(std::shared_future<GoalHandleCounterWithDelay::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        auto result_future = goal_handle->get_result_async();
        result_future.then(
            std::bind(&CounterWithDelayClient::get_result_callback, this, std::placeholders::_1));
    }

    void feedback_callback(GoalHandleCounterWithDelay::SharedPtr goal_handle, const std::shared_ptr<const CounterWithDelay::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %d", feedback->counts_elapsed);
    }

    void get_result_callback(std::shared_future<rclcpp_action::ClientGoalHandle<CounterWithDelay>::Result> future) {
        auto result = future.get();
        if (result) {
            RCLCPP_INFO(this->get_logger(), "Result: %s", result->result_message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result");
        }
        rclcpp::shutdown();
    }

    rclcpp_action::Client<CounterWithDelay>::SharedPtr client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto counter_with_delay_client = std::make_shared<CounterWithDelayClient>();

    counter_with_delay_client->send_goal(10);

    rclcpp::spin(counter_with_delay_client);
    rclcpp::shutdown();
    return 0;
}
