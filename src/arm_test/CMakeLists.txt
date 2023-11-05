cmake_minimum_required(VERSION 3.8)
project(arm_test)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

# Library creation
add_library(${PROJECT_NAME} SHARED
  src/arm_server.cpp
  include/arm_test/arm_server.hpp
  src/arm_rt_server.cpp
  include/arm_test/arm_rt_server.hpp
)

set(dependencies
  "rclcpp"
  "rclcpp_action"
  "moveit_ros_planning_interface"
  "moveit_core"
  "moveit_msgs"
  "moveit_ros_planning"
  "moveit_visual_tools"
  "geometry_msgs"
  "example_interfaces"
  "arm_interfaces"
  "moveit_servo"
  "std_srvs"
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Node registration
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "urc_arm::server::ArmPlanAndExecuteServer"
  EXECUTABLE ${PROJECT_NAME}_ArmPlanAndExecuteServer
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "urc_arm::server::ArmRTServer"
  EXECUTABLE ${PROJECT_NAME}_ArmRTServer
)

# Simple tests
add_executable(move src/test.cpp)
ament_target_dependencies(move ${dependencies})

# Install launch files.
install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install library
install(TARGETS
  ${PROJECT_NAME}
  move
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the copyright linker
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
