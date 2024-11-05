from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the `sim` argument with a default value of `false`
    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='true',
        description='Set to true to run in simulation mode'
    )

    # Group for real hardware mode (unless sim is true)
    if 0:
        real_hardware_group = GroupAction(
            actions=[
                Node(
                    package='rosserial_python',
                    executable='serial_node.py',
                    name='sensor_info_publisher',
                    parameters=[
                        {'port': '/dev/ttyACM0'},
                        {'baud': 57600}
                    ]
                )
            ],
            condition=UnlessCondition(LaunchConfiguration('sim'))
        )

    # Group for simulation mode (if sim is true)
    simulation_group = GroupAction(
        actions=[
            Node(
                package='range_sensor_cpp',
                executable='sensor_info_publisher_simulation',
                name='sensor_info_publisher'
            )
        ],
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    # Combine all launch actions
    return LaunchDescription([
        sim_arg,
        #real_hardware_group,
        simulation_group
    ])
