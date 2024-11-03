#include <memory>
#include <chrono>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/action/action_server.hpp"
#include "range_sensors_interfaces/action/counter_with_delay.hpp"

using namespace std::chrono_literals;

class CounterWithDelayServer : public rclcpp::Node {
public:
    using CounterWithDelay = range_sensors_interfaces::action::CounterWithDelay;
    using GoalHandleCounterWithDelay = rclcpp_action::ServerGoalHandle<CounterWithDelay>;

    CounterWithDelayServer() : Node("counter_with_delay_action_server") {
        // Declare the parameter with a default value if it doesn't exist
        this->declare_parameter("counter_delay", 1.0);

        this->action_server_ = rclcpp_action::create_server<CounterWithDelay>(
            this,
            "counter_with_delay_action_server",
            std::bind(&CounterWithDelayServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CounterWithDelayServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&CounterWithDelayServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<CounterWithDelay>::SharedPtr action_server_;

    void handle_accepted(const GoalHandleCounterWithDelay::SharedPtr goal_handle) {
        std::thread{std::bind(&CounterWithDelayServer::execute, this, goal_handle)}.detach();
    }

    void execute(const GoalHandleCounterWithDelay::SharedPtr goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");

        // Get the parameter value
        double counter_delay_value;
        this->get_parameter("counter_delay", counter_delay_value);

        RCLCPP_INFO(this->get_logger(), "Using %f seconds for counter delay.", counter_delay_value);

        // Variable to store feedback
        auto feedback = std::make_shared<CounterWithDelay::Feedback>();

        // Start executing the action
        for (int counter_idx = 0; counter_idx < goal_handle->get_goal()->num_counts; ++counter_idx) {
            // Check if the goal has been canceled
            if (goal_handle->is_canceling()) {
                goal_handle->canceled();
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Publish feedback
            feedback->counts_elapsed = counter_idx;
            goal_handle->publish_feedback(feedback);

            // Wait for counter_delay seconds before incrementing the counter
            std::this_thread::sleep_for(std::chrono::duration<double>(counter_delay_value));
        }

        goal_handle->succeed();

        RCLCPP_INFO(this->get_logger(), "Succeeded");
        auto result = std::make_shared<CounterWithDelay::Result>();
        result->result_message = "Successfully completed counting.";
        goal_handle->set_result(result);
    }

    rclcpp_action::CancelResponse handle_cancel(const GoalHandleCounterWithDelay::SharedPtr goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    rclcpp_action::GoalResponse handle_goal(const std::shared_ptr<rclcpp_action::Server<CounterWithDelay>::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with num_counts: %d", goal->num_counts);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto counter_with_delay_server = std::make_shared<CounterWithDelayServer>();
    rclcpp::spin(counter_with_delay_server);
    rclcpp::shutdown();
    return 0;
}
