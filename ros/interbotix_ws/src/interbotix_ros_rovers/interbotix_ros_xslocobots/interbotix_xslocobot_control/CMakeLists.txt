cmake_minimum_required(VERSION 3.5)
project(interbotix_xslocobot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python executables
install(
  PROGRAMS
    demos/python_ros2_api/bartender.py
    demos/python_ros2_api/combo_control.py
    demos/python_ros2_api/ee_cartesian_trajectory.py
    demos/python_ros2_api/ee_pose_components.py
    demos/python_ros2_api/ee_pose_matrix_control.py
    demos/python_ros2_api/gripper_control.py
    demos/python_ros2_api/joint_position_control.py
    demos/python_ros2_api/joint_pwm_control.py
    demos/python_ros2_api/move_base.py
    demos/python_ros2_api/pan_tilt_control.py
    demos/python_ros2_api/move_base_forward_action_client.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
