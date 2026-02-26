cmake_minimum_required(VERSION 3.16)

project(zenslam)

set(CMAKE_CXX_STANDARD 23)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

add_subdirectory(3rd_party/ugpm)  # Builds both ugpm and basalt_imu targets
add_subdirectory(zenslam_app)
add_subdirectory(zenslam_core)
add_subdirectory(zenslam_options)
add_subdirectory(zenslam_tests)