cmake_minimum_required(VERSION 3.14)
project(zenslam_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Using OpenCV: " ${OpenCV_VERSION})

find_package(Boost CONFIG REQUIRED COMPONENTS program_options)
message(STATUS "Using Boost: " ${Boost_VERSION})

add_executable(zenslam_app source/main.cpp
        source/folder_options.cpp
        source/mono_folder_reader.cpp
        source/mono_frame.cpp
        source/stereo_folder_reader.cpp
        source/stereo_frame.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_highgui opencv_core Boost::program_options)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
