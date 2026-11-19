cmake_minimum_required(VERSION 3.8)
project(shoot_belt_measurement)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sequence_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(shoot_belt_measurement_node
  src/shoot_belt_measurement.cpp
)
ament_target_dependencies(shoot_belt_measurement_node
  rclcpp
  rclcpp_action
  can_msgs
  sequence_interfaces
  std_msgs
  std_srvs
)

install(TARGETS
  shoot_belt_measurement_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
