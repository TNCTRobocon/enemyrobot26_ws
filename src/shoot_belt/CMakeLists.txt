cmake_minimum_required(VERSION 3.8)
project(shoot_belt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sequence_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ShootBeltCommand.msg"
  "srv/ManualCurrent.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(shoot_belt_sequence src/shoot_belt_sequence.cpp)
ament_target_dependencies(shoot_belt_sequence
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sequence_interfaces
)
target_link_libraries(shoot_belt_sequence "${cpp_typesupport_target}")

add_executable(shoot_belt_joy_test src/shoot_belt_joy_test.cpp)
ament_target_dependencies(shoot_belt_joy_test
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  sequence_interfaces
)

add_executable(shoot_belt_can_rx src/shoot_belt_can_rx.cpp)
ament_target_dependencies(shoot_belt_can_rx
  rclcpp
  can_msgs
  std_msgs
)

add_executable(shoot_belt_can_tx src/shoot_belt_can_tx.cpp)
ament_target_dependencies(shoot_belt_can_tx
  rclcpp
  can_msgs
  std_msgs
)
target_link_libraries(shoot_belt_can_tx "${cpp_typesupport_target}")

install(TARGETS
  shoot_belt_sequence
  shoot_belt_joy_test
  shoot_belt_can_rx
  shoot_belt_can_tx
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
