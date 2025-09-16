cmake_minimum_required(VERSION 3.14)

project(zenslam_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

add_executable(zenslam_app
    source/main.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
