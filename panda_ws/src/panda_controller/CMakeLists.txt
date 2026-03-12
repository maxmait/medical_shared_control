cmake_minimum_required(VERSION 3.8)
project(panda_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(safety_controller REQUIRED)

add_executable(panda_velocity_controller src/panda_velocity_controller.cpp)
ament_target_dependencies(panda_velocity_controller
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  kdl_parser
  orocos_kdl
  urdf
  rcl_interfaces
)

add_executable(tele_controller_input src/tele_controller_input.cpp)
ament_target_dependencies(tele_controller_input
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(shared_control_bridge src/shared_control_bridge.cpp)
ament_target_dependencies(shared_control_bridge
  rclcpp
  sensor_msgs
  geometry_msgs
  safety_controller
)


install(TARGETS
  panda_velocity_controller
  tele_controller_input
  shared_control_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/slider_controller.py
  ${PROJECT_NAME}/velocity_test.py
  ${PROJECT_NAME}/test_controller_input.py
  ${PROJECT_NAME}/test_controller_vibration.py
  ${PROJECT_NAME}/haptic_feedback_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()