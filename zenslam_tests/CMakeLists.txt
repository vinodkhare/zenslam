cmake_minimum_required(VERSION 3.14)

project(zenslam_tests)

find_package(Boost CONFIG REQUIRED COMPONENTS program_options filesystem)
message(STATUS "Found Boost: " ${Boost_VERSION})

# Use Catch2 from vcpkg
find_package(Catch2 CONFIG REQUIRED)
message(STATUS "Found Catch2: ${Catch2_VERSION}")

# Include Catch2 test discovery
include(Catch)

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

add_executable(${PROJECT_NAME} source/zenslam_tests.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_imgproc opencv_calib3d)
catch_discover_tests(${PROJECT_NAME})
