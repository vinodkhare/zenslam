cmake_minimum_required(VERSION 3.16)

project(zenslam_metal LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_OBJCXX_STANDARD 23)

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "zenslam_metal: Found OpenCV ${OpenCV_VERSION}")

find_package(spdlog CONFIG REQUIRED)
message(STATUS "zenslam_metal: Found spdlog ${spdlog_VERSION}")

add_library(${PROJECT_NAME}
    source/pyr_lk_factory.cpp
    source/pyr_lk.mm
)

target_include_directories(${PROJECT_NAME}
    PUBLIC include
    PRIVATE source
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    zenslam_core
    opencv_core
    opencv_video
    PRIVATE
    spdlog::spdlog
    "-framework Metal"
    "-framework Foundation"
)
