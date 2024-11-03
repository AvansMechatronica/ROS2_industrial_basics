
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the template publisher ROS node
        Node(
            package='range_sensor_cpp',
            executable='template_publisher_script',
            name='node_1',
            output='screen'
        ),
        
        # Start the template subscriber ROS node
        Node(
            package='range_sensor_cpp',
            executable='template_subscriber_script',
            name='node_2',
            output='screen'
        ),
    ])
