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
        # Core
        source/slam_options.cpp
        source/slam_thread.cpp
        source/options.cpp
        source/processor.cpp
        source/time_this.cpp
        
        # Frame & Types
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
        
        # Calibration
        source/calibration/calibration.cpp
        source/calibration/camera_calibration.cpp
        source/calibration/imu_calibration.cpp
        
        # Detection
        source/detection/detector.cpp
        source/detection/correspondence_utils.cpp
        
        # Tracking
        source/tracking/tracker.cpp
        source/tracking/tracking_utils.cpp
        
        # Matching
        source/matching/matcher.cpp
        source/matching/matching_utils.cpp
        
        # Mapping
        source/mapping/triangulator.cpp
        source/mapping/triangulation_utils.cpp
        source/mapping/keyframe_selector.cpp
        source/mapping/keyframe_database.cpp
        
        # Motion
        source/motion/motion_predictor.cpp
        source/motion/gravity_estimator.cpp
        source/motion/integrator.cpp
        
        # Optimization
        source/optimization/local_bundle_adjustment.cpp
        
        # Utils
        source/utils/utils.cpp
        source/utils/utils_opencv.cpp
        source/utils/utils_slam.cpp
        source/utils/utils_std.cpp
        source/utils/rigid_transform.cpp
        source/utils/pose.cpp
        source/utils/estimator.cpp
        
        # I/O
        source/io/folder_options.cpp
        source/io/folder_reader.cpp
        source/io/groundtruth.cpp
        source/io/verb.cpp
        
        # Pose Estimation
        source/pose_estimation/point_estimator.cpp
        source/pose_estimation/line_estimator.cpp
        source/pose_estimation/combined_estimator.cpp
        source/pose_estimation/pose_fusion.cpp
        
        # Place Recognition
        source/place_recognition/bow_vocabulary.cpp
        source/place_recognition/bow_database.cpp
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
