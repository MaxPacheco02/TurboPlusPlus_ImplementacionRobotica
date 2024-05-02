cmake_minimum_required(VERSION 3.10.0)

project( Opencv )
find_package( OpenCV REQUIRED )
find_package(jsoncpp REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( main src/finding_hue.cpp )

target_link_libraries( main ${OpenCV_LIBS} jsoncpp)
