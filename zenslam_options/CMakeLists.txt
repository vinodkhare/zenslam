cmake_minimum_required(VERSION 3.10)

project(zenslam_options)

# Install all files from the options folder to the output directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/options/ DESTINATION ${CMAKE_BINARY_DIR}/options)
