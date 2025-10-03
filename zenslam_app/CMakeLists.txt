cmake_minimum_required(VERSION 3.14)

project(zenslam_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Using OpenCV: " ${OpenCV_VERSION})

find_package(Boost CONFIG REQUIRED COMPONENTS program_options)
message(STATUS "Using Boost: " ${Boost_VERSION})

find_package(Microsoft.GSL CONFIG REQUIRED)
message(STATUS "Using Microsoft.GSL: " ${Microsoft.GSL_VERSION})

find_package(concurrentqueue CONFIG REQUIRED)
message(STATUS "Using concurrentqueue: " ${concurrentqueue_VERSION})

find_package(spdlog CONFIG REQUIRED)
message(STATUS "Using spdlog: " ${spdlog_VERSION})

find_package(yaml-cpp CONFIG REQUIRED)
message(STATUS "Using yaml-cpp: " ${yaml-cpp_VERSION})

add_executable(zenslam_app source/main.cpp
        source/application.cpp
        source/event.cpp
        source/folder_options.cpp
        source/grid_detector.cpp
        source/mono_folder_reader.cpp
        source/mono_frame.cpp
        source/slam_thread.cpp
        source/stereo_folder_reader.cpp
        source/stereo_frame.cpp
        source/utils.cpp
        source/verb.cpp
        source/viewer.cpp
        source/thread_safe.cpp
        source/options.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PRIVATE
        opencv_highgui
        opencv_core
        opencv_features2d
        opencv_imgproc
        Boost::program_options
        spdlog::spdlog
        concurrentqueue::concurrentqueue
        Microsoft.GSL::GSL
        yaml-cpp::yaml-cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
