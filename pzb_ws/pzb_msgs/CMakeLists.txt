cmake_minimum_required(VERSION 3.8)
project(pzb_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generar interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Signal.msg"
  "msg/TrailerBearing.msg"
  "msg/ArucoMarker.msg"
  "msg/ArucoList.msg"
  "msg/LiftState.msg"
  "msg/VoiceCommand.msg"
  "srv/IsTrailerOnWatch.srv"
  "srv/TrailerAlign.srv"

  DEPENDENCIES std_msgs geometry_msgs geometry_msgs)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
