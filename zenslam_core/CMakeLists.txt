cmake_minimum_required(VERSION 3.14)

project(zenslam_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

find_package(Boost CONFIG REQUIRED COMPONENTS program_options)
message(STATUS "Using Boost: " ${Boost_VERSION})

find_package(Microsoft.GSL CONFIG REQUIRED)
message(STATUS "Using Microsoft.GSL: " ${Microsoft.GSL_VERSION})

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Using OpenCV: " ${OpenCV_VERSION})

find_package(VTK REQUIRED)
message(STATUS "Using VTK: " ${VTK_VERSION})

find_package(concurrentqueue CONFIG REQUIRED)
message(STATUS "Using concurrentqueue: " ${concurrentqueue_VERSION})

find_package(magic_enum CONFIG REQUIRED)
message(STATUS "Using magic_enum: " ${magic_enum_VERSION})

find_package(spdlog CONFIG REQUIRED)
message(STATUS "Using spdlog: " ${spdlog_VERSION})

find_package(yaml-cpp CONFIG REQUIRED)
message(STATUS "Using yaml-cpp: " ${yaml-cpp_VERSION})

add_library(${PROJECT_NAME} 
        source/calibration.cpp
        source/event.cpp
        source/folder_options.cpp
        source/grid_detector.cpp
        source/keypoint.cpp
        source/match_data.cpp
        source/mono_folder_reader.cpp
        source/mono_frame.cpp
        source/options.cpp
        source/point.cpp
        source/slam_thread.cpp
        source/stereo_folder_reader.cpp
        source/stereo_frame.cpp
        source/thread_safe.cpp
        source/utils.cpp
        source/utils_std.cpp
        source/verb.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include/zenslam PUBLIC include)
target_link_libraries(${PROJECT_NAME} PRIVATE
        opencv_calib3d
        opencv_core
        opencv_features2d
        opencv_imgproc
        opencv_video
        Boost::program_options
        spdlog::spdlog
        concurrentqueue::concurrentqueue
        Microsoft.GSL::GSL
        yaml-cpp::yaml-cpp
        magic_enum::magic_enum
        ${VTK_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
