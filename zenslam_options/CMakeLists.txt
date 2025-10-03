cmake_minimum_required(VERSION 3.10)

project(zenslam_options)

# Install all files from the options folder to the output directory
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/options/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/options
        FILES_MATCHING PATTERN "*")