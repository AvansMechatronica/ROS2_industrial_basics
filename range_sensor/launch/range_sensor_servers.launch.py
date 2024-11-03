from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument
    counter_delay_parameter = DeclareLaunchArgument(
        'counter_delay_parameter', 
        default_value='1.0'
    )

    # Define the metres_to_feet service server node
    metres_to_feet_node = Node(
        package='range_sensor',
        executable='metres_to_inches_server',
        name='metres_to_inches',
        output='screen'
    )

    # Define the counter_with_delay action server node with parameter
    counter_with_delay_node = Node(
        package='range_sensor',
        executable='counter_with_delay_action_server',
        name='counter_with_delay',
        output='screen',
        parameters=[{
            'counter_delay': LaunchConfiguration('counter_delay_parameter')
        }]
    )

    # Combine all launch actions
    return LaunchDescription([
        counter_delay_parameter,
        metres_to_feet_node,
        counter_with_delay_node
    ])
