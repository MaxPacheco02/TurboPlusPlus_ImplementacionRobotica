cmake_minimum_required(VERSION 3.10.0)

project( test_cmake )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( test test.cpp )
#add_executable( test_extrinsic_camera_params test_extrinsic_camera_params.cpp )

target_link_libraries( test ${OpenCV_LIBS} )
#target_link_libraries( test_extrinsic_camera_params ${OpenCV_LIBS})
