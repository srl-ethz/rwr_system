cmake_minimum_required(VERSION 3.15)
project(franka_msgs)

# Prefer this package's own generated libraries over similarly-named
# libraries that might exist in the base environment (e.g. conda).
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/../../..")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Grasp.action"
  "action/Homing.action"
  "action/Move.action"
  "srv/ErrorRecovery.srv"
  "srv/SetCartesianStiffness.srv"
  "srv/SetTCPFrame.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointStiffness.srv"
  "srv/SetStiffnessFrame.srv"
  "srv/SetLoad.srv"
  "srv/SetMujocoPoses.srv"
  "msg/GraspEpsilon.msg"
  "msg/FrankaState.msg"
  "msg/FrankaModel.msg"
  "msg/Errors.msg"
  "msg/PoseStampedArray.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
