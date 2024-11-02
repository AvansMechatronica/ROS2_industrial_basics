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
    # create messages that are used to publish feedback/result
    _feedback = CounterWithDelay.Feedback()
    _result = CounterWithDelay.Result()

    def __init__(self):
        super().__init__('counter_with_delay_action_server')
        self._action_server = ActionServer(
            self,
            CounterWithDelay,
            'counter_with_delay_action_server',
            self.action_server_callback)

    def action_server_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        counter_delay_value = 1.0
        ## Assignment 3 - Part3
        ## modify counter delay using a private parameter .
        ## Uncomment the following lines (37-41) and modify them acordingly

        #if rospy.has_param("/counter_with_delay/counter_delay"):
        #    counter_delay_value = rospy.get_param("/counter_with_delay/counter_delay")
        #    rospy.loginfo("Parameter %s was found on the parameter server. Using %fs for counter delay."%("/counter_with_delay/counter_delay", counter_delay_value))
        #else:
        #    rospy.loginfo("Parameter %s not found on the parameter server. Using default value of 1.0s for counter delay.","counter_delay")

        ##  End of Assignment 3 - Part3

        # Variable for delay
        #delay_rate = rospy.Rate(counter_delay_value)
        delay_rate = 100

        # Variable to decide the final state of the action server.
        #success = True

        # publish info to the console for the user
        self.get_logger().info('%s action server is counting up to  %i with %fs delay between each count' % (self._action_name, goal.num_counts, counter_delay_value))

        # start executing the action
        for counter_idx in range(0, goal_handle.request.num_counts):
            # check that preempt has not been requested by the client

            # publish the feedback
            self._feedback.counts_elapsed = counter_idx
            goal_handle.publish_feedback(self._feedback)
            # Wait for counter_delay s before incrementing the counter.
            time.sleep(1)


        result = CounterWithDelay.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    counter_with_delay_server = CounterWithDelayServer()

    rclpy.spin(counter_with_delay_server)


if __name__ == '__main__':
    main()


