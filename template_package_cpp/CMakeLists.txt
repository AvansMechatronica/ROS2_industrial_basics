cmake_minimum_required(VERSION 3.8)
project(template_package_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Add this if your node uses rclcpp

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
  TARGETS template_publisher
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
