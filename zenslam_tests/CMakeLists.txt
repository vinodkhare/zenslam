cmake_minimum_required(VERSION 3.14)

project(zenslam_tests)

# Use Catch2 from vcpkg
find_package(Catch2 CONFIG REQUIRED)
message(STATUS "Using Catch2: ${Catch2_VERSION}")

add_executable(hello_test source/hello_test.cpp)
target_link_libraries(hello_test PRIVATE Catch2::Catch2WithMain)

enable_testing()
add_test(NAME hello_test COMMAND hello_test)
