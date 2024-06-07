cmake_minimum_required(VERSION 3.8)
project(pzb_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
# Custom messages
find_package(pzb_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

add_executable(color_detection src/color_detection.cpp)
ament_target_dependencies(color_detection rclcpp std_msgs OpenCV sensor_msgs cv_bridge pzb_msgs)

add_executable(camera_view src/camera_view.cpp)
ament_target_dependencies(camera_view rclcpp std_msgs OpenCV sensor_msgs cv_bridge pzb_msgs)

install(TARGETS
    color_detection
    camera_view
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables

install(DIRECTORY
    scripts
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/signal_detection_node.py
    scripts/line_detection_node.py
    scripts/line_detection_node2.py
    scripts/dotted_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
