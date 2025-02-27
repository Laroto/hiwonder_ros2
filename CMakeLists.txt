cmake_minimum_required(VERSION 3.5)
project(hiwonder_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# Include directories
include_directories(
  include
  include/serial_bus_servo_controller_cpp/include
)

set (hiwonder_services
  "srv/GetMotorPositions.srv"
)

# Generate custom service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "${hiwonder_services}"
)

# Ensure service message headers are built before the C++ code
ament_export_dependencies(rosidl_default_runtime)

# Add the executable
add_executable(hiwonder_ros_node src/hiwonder_ros.cpp)
add_executable(write_only src/write_motors_only.cpp)
add_executable(service_server src/service_server.cpp)

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

rosidl_target_interfaces(service_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(service_server
  rclcpp
)

# Install targets
install(TARGETS
  service_server
  hiwonder_ros_node
  write_only
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
