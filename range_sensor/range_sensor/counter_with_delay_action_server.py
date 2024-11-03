#! /usr/bin/env python

# This code has been adapted from the ROS Wiki actionlib tutorials to the context
# of this course.
# (http://wiki.ros.org/range_sensor_msgs/Tutorials)

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from range_sensors_interfaces.action import CounterWithDelay

class CounterWithDelayServer(Node):

    def __init__(self):
        super().__init__('counter_with_delay_action_server')
        self._action_name = 'counter_with_delay_action_server'
        self._action_server = ActionServer(
            self,
            CounterWithDelay,
            self._action_name,
            self.action_server_callback)

    def action_server_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        ## Assignment 3 - Part3
        ## modify counter delay using a private parameter .
        ## Uncomment the following lines (37-41) and modify them acordingly

        # Declare the parameter with a default value if it doesn't exist
        self.declare_parameter('counter_delay', 1.0)
        
        # Get the parameter value
        counter_delay_value = self.get_parameter('counter_delay').value
        
        # Check if the parameter was set explicitly or is using the default
        if self.get_parameter('counter_delay').get_parameter_value().type == 4:  # 4 means PARAMETER_NOT_SET
            self.get_logger().info(
                "Parameter 'counter_delay' not found on the parameter server. Using default value of 1.0s for counter delay."
            )
            counter_delay_value = 1.0
        else:
            self.get_logger().info(
                f"Parameter 'counter_delay' was found on the parameter server. Using {counter_delay_value}s for counter delay."
            )


        ##  End of Assignment 3 - Part3

        # Variable for delay
        #delay_rate = rospy.Rate(counter_delay_value)
        delay_rate = 100

        # Variable to decide the final state of the action server.
        #success = True

        # publish info to the console for the user
        self.get_logger().info('%s action server is counting up to  %i with %fs delay between each count' % (self._action_name, goal_handle.request.num_counts, counter_delay_value))

        feedback = CounterWithDelay.Feedback()

        # start executing the action
        for counter_idx in range(0, goal_handle.request.num_counts):
            # check that preempt has not been requested by the client

            # publish the feedback
            feedback.counts_elapsed = counter_idx
            goal_handle.publish_feedback(feedback)
            # Wait for counter_delay s before incrementing the counter.
            time.sleep(1)

        goal_handle.succeed()

        self.get_logger().info('%s: Succeeded' % self._action_name)
        result = CounterWithDelay.Result()
        result.result_message = "Successfully completed counting."
        return result


def main(args=None):
    rclpy.init(args=args)

    counter_with_delay_server = CounterWithDelayServer()

    rclpy.spin(counter_with_delay_server)


if __name__ == '__main__':
    main()


