cmake_minimum_required(VERSION 3.5)
project(hiwonder_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  include/serial_bus_servo_controller_cpp/include
)

# Add the executable
add_executable(hiwonder_ros_node src/hiwonder_ros.cpp)

# Link libraries
ament_target_dependencies(hiwonder_ros_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  hiwonder_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
