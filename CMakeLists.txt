cmake_minimum_required(VERSION 3.5)
project(hiwonder_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)

# Include directories
include_directories(
  include
  include/serial_bus_servo_controller_cpp/include
)

# Add the executable
add_executable(hiwonder_ros_node src/hiwonder_ros.cpp)

add_executable(write_only src/write_motors_only.cpp)

# Link libraries
ament_target_dependencies(hiwonder_ros_node
  rclcpp
  sensor_msgs
  actuator_msgs
)

ament_target_dependencies(write_only
  rclcpp
  sensor_msgs
  actuator_msgs
)

install(TARGETS
  hiwonder_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  write_only
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
