#include <memory>
#include <chrono>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "range_sensors_interfaces/action/counter_with_delay.hpp"

using namespace std::chrono_literals;

class CounterWithDelayServer : public rclcpp::Node {
public:
    using CounterWithDelay = range_sensors_interfaces::action::CounterWithDelay;
    using GoalHandleCounterWithDelay = rclcpp_action::ServerGoalHandle<CounterWithDelay>;

  explicit CounterWithDelayServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("counter_with_delay_action_server", options)
  {
    using namespace std::placeholders;
#if 0
    this->action_server_ = rclcpp_action::create_server<CounterWithDelay>(
      this,
      "counter_with_delay_action_server",
      std::bind(&CounterWithDelayServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CounterWithDelayServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CounterWithDelayServer::handle_accepted, this, std::placeholders::_1));
#endif
  }

  
private:
    rclcpp_action::Server<CounterWithDelay>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const std::shared_ptr<GoalHandleCounterWithDelay> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with number of counts %d", goal_handle->get_goal()->num_counts);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCounterWithDelay> goal_handle) {
        std::thread{std::bind(&CounterWithDelayServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleCounterWithDelay> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Executing goal...");

        // Get the parameter value
        double counter_delay_value;
        this->get_parameter("counter_delay", counter_delay_value);

        RCLCPP_INFO(this->get_logger(), "Using %f seconds for counter delay.", counter_delay_value);

        // Variable to store feedback
        auto feedback = std::make_shared<CounterWithDelay::Feedback>();
        auto result = std::make_shared<CounterWithDelay::Result>();

        // Start executing the action
        for (int counter_idx = 0; counter_idx < goal_handle->get_goal()->num_counts; ++counter_idx) {
            // Check if the goal has been canceled
            if (goal_handle->is_canceling()) {
                result->result_message = "Canceled counting.";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Publish feedback
            feedback->counts_elapsed = counter_idx;
            goal_handle->publish_feedback(feedback);

            // Wait for counter_delay seconds before incrementing the counter
            std::this_thread::sleep_for(std::chrono::duration<double>(counter_delay_value));
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
            result->result_message = "Successfully completed counting.";
            goal_handle->succeed(result);
        }
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCounterWithDelay> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto counter_with_delay_server = std::make_shared<CounterWithDelayServer>();
    rclcpp::spin(counter_with_delay_server);
    rclcpp::shutdown();
    return 0;
}

