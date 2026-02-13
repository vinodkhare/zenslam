cmake_minimum_required(VERSION 3.16)

project(zenslam_core)

set(CMAKE_CXX_STANDARD 23)

find_package(concurrentqueue CONFIG REQUIRED)
message(STATUS "Found concurrentqueue: " ${concurrentqueue_VERSION})

find_package(magic_enum CONFIG REQUIRED)
message(STATUS "Found magic_enum: " ${magic_enum_VERSION})

find_package(nanoflann CONFIG REQUIRED)
message(STATUS "Found nanoflann: " ${nanoflann_VERSION})

find_package(spdlog CONFIG REQUIRED)
message(STATUS "Found spdlog: " ${spdlog_VERSION})

find_package(yaml-cpp CONFIG REQUIRED)
message(STATUS "Found yaml-cpp: " ${yaml-cpp_VERSION})

find_package(Boost CONFIG REQUIRED COMPONENTS program_options filesystem)
message(STATUS "Found Boost: " ${Boost_VERSION})

# Ceres is required by ugpm headers (even if using LPM)
find_package(Ceres REQUIRED)
message(STATUS "Found Ceres: ${Ceres_VERSION}")

find_package(Microsoft.GSL CONFIG REQUIRED)
message(STATUS "Found Microsoft.GSL: " ${Microsoft.GSL_VERSION})

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Found OpenCV: " ${OpenCV_VERSION})

find_package(VTK REQUIRED)
message(STATUS "Found VTK: " ${VTK_VERSION})

find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")


add_library(${PROJECT_NAME}
        source/frame/counts.cpp
        source/frame/durations.cpp
        source/frame/sensor.cpp
        source/frame/writer.cpp
        source/types/event.cpp
        source/types/keyline.cpp
        source/types/keypoint.cpp
        source/types/line3d.cpp
        source/types/map.cpp
        source/types/point3d.cpp
        source/calibration.cpp
        source/camera_calibration.cpp
        source/detector.cpp
        source/folder_options.cpp
        source/slam_options.cpp
        source/groundtruth.cpp
        source/imu_calibration.cpp
        source/integrator.cpp
        source/estimator.cpp
        source/keyframe_selector.cpp
        source/pose_estimation/point_estimator.cpp
        source/pose_estimation/line_estimator.cpp
        source/pose_estimation/combined_estimator.cpp
        source/pose_estimation/pose_fusion.cpp
        source/triangulator.cpp
        source/matcher.cpp
        source/motion_predictor.cpp
        source/gravity_estimator.cpp
        source/options.cpp
        source/pose.cpp
        source/processor.cpp
        source/slam_thread.cpp
        source/folder_reader.cpp
        source/time_this.cpp
        source/tracker.cpp
        source/utils.cpp
        source/utils_opencv.cpp
        source/utils_slam.cpp
        source/utils_std.cpp
        source/verb.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${RAPIDCSV_INCLUDE_DIRS} include)

target_link_libraries(${PROJECT_NAME} PUBLIC
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
        Boost::filesystem
        Boost::program_options
        Ceres::ceres
        Microsoft.GSL::GSL
        ${VTK_LIBRARIES}
        ugpm
)

# Add ugpm headers (preint) to include path
target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

# Enable precompiled headers for zenslam_core only
target_precompile_headers(${PROJECT_NAME} PRIVATE include/zenslam/pch.h)
