import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Brings in the messages used by the CounterWithDelay action, including the
# goal message and the result message.
from range_sensors_interfaces.action import CounterWithDelay



class CounterWithDelayClient(Node):

    def __init__(self):
        super().__init__('counter_with_delay_action_client')
        self._counter_with_delay_client = ActionClient(self, CounterWithDelay, 'counter_with_delay_action_server')

    def send_goal(self, num_counts):
        goal_msg = CounterWithDelay.Goal()
        goal_msg.num_counts = num_counts

        self._counter_with_delay_client.wait_for_server()
        self._send_goal_future = self._counter_with_delay_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.counts_elapsed))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result_message))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    counter_with_delay_client = CounterWithDelayClient()

    counter_with_delay_client.send_goal(10)

    rclpy.spin(counter_with_delay_client)


if __name__ == '__main__':
    main()








if 0:
    def counter_with_delay_client():
        # Creates the SimpleActionClient, passing the type of the action
        # (CounterWithDelayAction) to the constructor.
        client = actionlib.SimpleActionClient('counter_with_delay', CounterWithDelayAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.get_logger().info("Waiting for action server to come up...")
        client.wait_for_server()

        num_counts = 3

        # Creates a goal to send to the action server.
        goal = CounterWithDelayGoal(num_counts)

        # Sends the goal to the action server.
        client.send_goal(goal)

        self.get_logger().info("Goal has been sent to the action server.")

        # Waits for the server to finish performing the action.
        # client.wait_for_result()

        # Does something else while the action is being done:
        for count_idx in range(0, num_counts):
            self.get_logger().info('I am doing other things while the goal is being serviced by the server')
            rospy.sleep(1.2)

        # Prints out the result of executing the action
        return client.get_result()  # A CounterWithDelayResult

    if __name__ == '__main__':
        try:
            # Initializes a rospy node so that the SimpleActionClient can
            # publish and subscribe over ROS.
            rospy.init_node('counter_with_delay_ac')
            result = counter_with_delay_client()
            self.get_logger().info(result.result_message)
        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)
