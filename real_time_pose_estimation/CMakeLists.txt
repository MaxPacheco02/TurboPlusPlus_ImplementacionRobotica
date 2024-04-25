cmake_minimum_required(VERSION 3.10.0)
project( real_time_pose_estimation )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(sample_dir src/)
set(target example_tutorial_)

set(sample_pnplib
        ${sample_dir}CsvReader.cpp
        ${sample_dir}CsvWriter.cpp
        ${sample_dir}ModelRegistration.cpp
        ${sample_dir}Mesh.cpp
        ${sample_dir}Model.cpp
        ${sample_dir}PnPProblem.cpp
        ${sample_dir}Utils.cpp
        ${sample_dir}RobustMatcher.cpp
)


add_executable( ${target}pnp_registration ${sample_dir}main_registration.cpp ${sample_pnplib} )
add_executable( ${target}pnp_detection ${sample_dir}main_detection.cpp ${sample_pnplib} )

target_link_libraries( ${target}pnp_registration ${OpenCV_LIBS})
target_link_libraries( ${target}pnp_detection ${OpenCV_LIBS})


