cmake_minimum_required(VERSION 3.14)

project(ugpm LANGUAGES CXX)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)



