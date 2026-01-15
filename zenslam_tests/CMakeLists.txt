cmake_minimum_required(VERSION 3.14)

project(zenslam_tests)

find_package(Boost CONFIG REQUIRED COMPONENTS program_options filesystem)
message(STATUS "Found Boost: " ${Boost_VERSION})

# Use Catch2 from vcpkg
find_package(Catch2 CONFIG REQUIRED)
message(STATUS "Found Catch2: ${Catch2_VERSION}")

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

add_executable(hello_test source/hello_test.cpp)
target_link_libraries(hello_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_features2d opencv_imgproc)

enable_testing()
add_test(NAME hello_test COMMAND hello_test)

add_executable(umeyama_test source/umeyama_test.cpp)
target_link_libraries(umeyama_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_imgproc)
add_test(NAME umeyama_test COMMAND umeyama_test)

add_executable(imu_calibration_test source/imu_calibration_test.cpp)
target_link_libraries(imu_calibration_test PRIVATE zenslam_core Catch2::Catch2WithMain)
add_test(NAME imu_calibration_test COMMAND imu_calibration_test)

add_executable(options_test source/options_test.cpp)
target_link_libraries(options_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_imgproc)
add_test(NAME options_test COMMAND options_test)

add_executable(stereo_rectification_test source/stereo_rectification_test.cpp)
target_link_libraries(stereo_rectification_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_imgproc opencv_calib3d)
add_test(NAME stereo_rectification_test COMMAND stereo_rectification_test)
