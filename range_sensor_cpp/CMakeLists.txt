cmake_minimum_required(VERSION 3.8)
project(range_sensor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Add this if your node uses rclcpp
find_package(range_sensors_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# Add the executable targets
add_executable(sensor_info_publisher_simulation src/sensor_info_publisher_simulation.cpp)
ament_target_dependencies(sensor_info_publisher_simulation 
  rclcpp
  range_sensors_interfaces 
  sensor_msgs
)

add_executable(sensor_info_subscriber src/sensor_info_subscriber.cpp)
ament_target_dependencies(sensor_info_subscriber 
  rclcpp
  range_sensors_interfaces 
  sensor_msgs
)

add_executable(metres_to_inches_server src/metres_to_inches_server.cpp)
ament_target_dependencies(metres_to_inches_server 
  rclcpp
  range_sensors_interfaces 
  sensor_msgs
  
)

add_executable(metres_to_inches_client src/metres_to_inches_client.cpp)
ament_target_dependencies(metres_to_inches_client 
  rclcpp
  range_sensors_interfaces 
  sensor_msgs
)

#add_executable(counter_with_delay_action_server src/counter_with_delay_action_server.cpp)
#ament_target_dependencies(counter_with_delay_action_server 
#  rclcpp
#  range_sensors_interfaces 
#  sensor_msgs
#  rclcpp_action
#  rclcpp_components
#)

#add_executable(counter_with_delay_action_client src/counter_with_delay_action_client.cpp)
#ament_target_dependencies(counter_with_delay_action_client 
#  rclcpp
#  range_sensors_interfaces 
#  sensor_msgs
#  rclcpp_action
#)

add_executable(assignment1 src/assignment1.cpp)
ament_target_dependencies(assignment1 
  rclcpp
  range_sensors_interfaces 
  sensor_msgs
)

add_executable(assignment2 src/assignment2.cpp)
ament_target_dependencies(assignment2 
  rclcpp
  range_sensors_interfaces 
  sensor_msgs
)

add_executable(template_publisher src/template_publisher.cpp)
ament_target_dependencies(template_publisher
  rclcpp
  std_msgs
)

add_executable(template_subscriber src/template_subscriber.cpp)
ament_target_dependencies(template_subscriber
  rclcpp
  std_msgs
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS sensor_info_publisher_simulation
          sensor_info_subscriber
          metres_to_inches_server
          metres_to_inches_client
#          counter_with_delay_action_server
#          counter_with_delay_action_client
          assignment1
          assignment2
          template_publisher
          template_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing and linting if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
