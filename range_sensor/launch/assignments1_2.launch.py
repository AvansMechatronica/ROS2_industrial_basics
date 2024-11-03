from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the `sim` argument with a default value of `false`
    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='true', # Shuold be false
        description='Simulation mode'
    )

    # Path to the sensor_info_publisher launch file
    sensor_info_publisher_launch_path = os.path.join(
        get_package_share_directory('range_sensor'),
        'launch',
        'sensor_info_publisher.launch.py'  # assuming converted to Python
    )

    # Include the sensor_info_publisher launch file with the `sim` argument
    sensor_info_publisher_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_info_publisher_launch_path),
        launch_arguments={'sim': LaunchConfiguration('sim')}.items()
    )

    # Define the BoxHeightInformation publisher node (Assignment 1)
    box_height_metres_node = Node(
        package='range_sensor',
        executable='assignment1',
        name='box_height_metres',
        output='screen'
    )

    # Define the metres_to_feet service server node
    metres_to_feet_node = Node(
        package='range_sensor',
        executable='metres_to_inches_server',
        name='metres_to_inches',
        output='screen'
    )

      # Define the BoxHeightInformation subscriber / convert to feet node (Assignment 2)
    box_height_feet_node = Node(
        package='range_sensor',
        executable='assignment2',
        name='box_height_feet',
        output='screen'
    )

    # Combine all launch actions
    return LaunchDescription([
        sim_arg,
        sensor_info_publisher_include,
        box_height_metres_node,
        metres_to_feet_node,
        box_height_feet_node
    ])
