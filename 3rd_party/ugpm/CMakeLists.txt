cmake_minimum_required(VERSION 3.14)

project(ugpm LANGUAGES CXX)

# Original UGPM library (header-only)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)

# Basalt IMU preintegration (header-only, copied locally)
# Uses vcpkg-provided Eigen and cereal
# Sophus is bundled (not available in vcpkg)
add_library(basalt_imu INTERFACE)
target_include_directories(basalt_imu INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../basalt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../basalt/thirdparty
)



