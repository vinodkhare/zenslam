cmake_minimum_required(VERSION 3.14)

project(zenslam_tests)

# Use Catch2 from vcpkg
find_package(Catch2 CONFIG REQUIRED)
message(STATUS "Found Catch2: ${Catch2_VERSION}")

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

add_executable(hello_test source/hello_test.cpp)
target_link_libraries(hello_test PRIVATE zenslam_core Catch2::Catch2WithMain)

enable_testing()
add_test(NAME hello_test COMMAND hello_test)

add_executable(umeyama_test source/umeyama_test.cpp)
target_link_libraries(umeyama_test PRIVATE zenslam_core Catch2::Catch2WithMain opencv_core opencv_imgproc)
add_test(NAME umeyama_test COMMAND umeyama_test)
