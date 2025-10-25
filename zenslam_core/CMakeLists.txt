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

find_package(nanoflann CONFIG REQUIRED)
message(STATUS "Using nanoflann: " ${nanoflann_VERSION})

find_package(spdlog CONFIG REQUIRED)
message(STATUS "Using spdlog: " ${spdlog_VERSION})

find_package(yaml-cpp CONFIG REQUIRED)
message(STATUS "Using yaml-cpp: " ${yaml-cpp_VERSION})

find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_library(${PROJECT_NAME}
        source/frame/camera.cpp
        source/frame/counts.cpp
        source/frame/durations.cpp
        source/frame/sensor.cpp
        source/frame/slam.cpp
        source/frame/stereo.cpp
        source/frame/writer.cpp
        source/types/event.cpp
        source/types/keyline.cpp
        source/types/keypoint.cpp
        source/types/line3d.cpp
        source/types/map.cpp
        source/types/point3d.cpp
        source/calibration.cpp
        source/camera_calibration.cpp
        source/folder_options.cpp
        source/grid_detector.cpp
        source/groundtruth.cpp
        source/imu_calibration.cpp
        source/motion.cpp
        source/options.cpp
        source/pose.cpp
        source/slam_thread.cpp
        source/stereo_folder_reader.cpp
        source/time_this.cpp
        source/utils.cpp
        source/utils_opencv.cpp
        source/utils_slam.cpp
        source/utils_std.cpp
        source/verb.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include/zenslam ${RAPIDCSV_INCLUDE_DIRS} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PRIVATE
        concurrentqueue::concurrentqueue
        magic_enum::magic_enum
        nanoflann::nanoflann
        opencv_calib3d
        opencv_core
        opencv_features2d
        opencv_imgproc
        opencv_line_descriptor
        opencv_video
        opencv_xfeatures2d
        spdlog::spdlog
        yaml-cpp::yaml-cpp
        Boost::program_options
        Microsoft.GSL::GSL
        ${VTK_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
