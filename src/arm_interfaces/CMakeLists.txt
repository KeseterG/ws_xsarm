cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_servo REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    "action/Plan.action"
    "action/Execute.action"
    "action/Enable.action"
    "action/Shutdown.action"
    "srv/RTCommand.srv"
  DEPENDENCIES
    std_msgs
    moveit_msgs
    control_msgs
    geometry_msgs
    moveit_servo
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
