cmake_minimum_required(VERSION 3.8)
project(autodrive_slam_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python nodes as executable scripts
install(PROGRAMS
  scripts/autodrive_adapter_node.py
  scripts/ackermann_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files (yaml)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
