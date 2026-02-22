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

add_executable(hello_test source/hello_test.cpp)
target_link_libraries(hello_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_features2d opencv_imgproc)
catch_discover_tests(hello_test)

add_executable(umeyama_test source/umeyama_test.cpp)
target_link_libraries(umeyama_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_imgproc)
catch_discover_tests(umeyama_test)

add_executable(imu_calibration_test source/imu_calibration_test.cpp)
target_link_libraries(imu_calibration_test PRIVATE zenslam_core Catch2::Catch2WithMain)
catch_discover_tests(imu_calibration_test)

add_executable(stereo_rectification_test source/stereo_rectification_test.cpp)
target_link_libraries(stereo_rectification_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_imgproc opencv_calib3d)
catch_discover_tests(stereo_rectification_test)
