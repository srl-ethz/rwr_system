cmake_minimum_required(VERSION 3.15)
project(experiments)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  model_inference_node.py
  calibrate_camera_pose.py
  calibrate_franka_load.py
  run_teleop_rokoko.py
  gesture_runner.py
  keyboard_runner.py
  run_keyboard_teleop_franka.py
  rl_inference/cube_detection_node.py
  rl_inference/policy_inference_cube_spin_v2.py
  command_gui.py
  DESTINATION lib/${PROJECT_NAME}
)


# install the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  cfgs
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
