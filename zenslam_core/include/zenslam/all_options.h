#pragma once

#include <filesystem>
#include <string>

#include <opencv2/core/types.hpp>
#include <spdlog/common.h>

#include "detection/detection_options.h"

#include "zenslam/io/verb.h"
#include "zenslam/motion/integrator.h"

namespace zenslam
{
    enum class matcher_type
    {
        BRUTE,
        KNN,
        FLANN
    };

    // ========================================================================
    // Detection and Feature Extraction
    // ========================================================================


    // ========================================================================
    // Tracking and Optical Flow
    // ========================================================================

    struct tracking_options
    {
        cv::Size klt_window_size         = cv::Size(31, 31);
        int      klt_max_level           = 3;
        double   klt_threshold           = 1.0;
        double   klt_min_tracked_ratio   = 0.6;
        double   landmark_match_distance = 32.0; // Descriptor distance threshold for matching keypoints to landmarks
        double   landmark_match_radius   = 50.0; // Radius in meters for landmark matching around the camera
        bool     use_keylines            = true; // Enable keyline tracking and triangulation
    };

    // ========================================================================
    // Triangulation and 3D Reconstruction
    // ========================================================================

    struct triangulation_options
    {
        double min_disparity          = 2.0;
        int    keyline_mask_margin    = 10;
        double min_angle              = 15.0;
        double reprojection_threshold = 1.0;
        double min_depth              = 1.0;
        double max_depth              = 50.0;
    };

    // ========================================================================
    // Keyframe Selection
    // ========================================================================

    struct keyframe_options
    {
        int    min_frames        = 5;
        int    max_frames        = 30;
        double min_translation   = 0.15;
        double min_rotation_deg  = 10.0;
        double min_tracked_ratio = 0.6;
        int    min_inliers       = 20;
    };

    // ========================================================================
    // Local Bundle Adjustment
    // ========================================================================

    struct lba_options
    {
        int    max_iterations   = 30;
        double huber_delta      = 1.0;
        bool   refine_landmarks = true;
    };

    // ========================================================================
    // PnP Pose Estimation
    // ========================================================================

    struct pnp_options
    {
        int    iterations             = 1000;
        float  threshold              = 3;
        double confidence             = 0.99;
        bool   use_refinement         = false;
        int    min_refinement_inliers = 4;
    };

    // ========================================================================
    // Essential Matrix Estimation
    // ========================================================================

    struct essential_options
    {
        double confidence  = 0.999;
        double threshold   = 1.0;
        int    min_inliers = 5;
    };

    // ========================================================================
    // 3D-3D Rigid Transformation
    // ========================================================================

    struct rigid_options
    {
        double threshold           = 0.1;
        int    iterations          = 1000;
        int    min_correspondences = 3;
    };

    // ========================================================================
    // SLAM Configuration (aggregates all sub-modules)
    // ========================================================================

    struct slam_options
    {
        // Feature detection and matching
        matcher_type       matcher           = matcher_type::BRUTE;
        double             matcher_ratio     = 0.8;
        integrator::method integrator_method = integrator::method::ugpm;

        // Pose estimation thresholds
        double threshold_3d3d              = 0.005;
        double threshold_3d2d              = 1.0;
        double reprojection_threshold_3d2d = 2.0;

        // Epipolar and frustum constraints
        double epipolar_threshold     = 1.0;
        bool   enable_frustum_culling = true;
        double frustum_margin         = 50.0;

        // Nested module configurations
        detection_options     detection;
        tracking_options      tracking;
        triangulation_options triangulation;
        keyframe_options      keyframe;
        lba_options           lba;
        pnp_options           pnp;
        essential_options     essential;
        rigid_options         rigid;
    };

    // ========================================================================
    // GUI and Visualization
    // ========================================================================

    struct gui_options
    {
        bool       show_keypoints       = true;
        bool       show_keylines        = true;
        cv::Scalar keyline_single_color = cv::Scalar(0, 255, 255); // BGR
        cv::Scalar keyline_match_color  = cv::Scalar(0, 255, 0);   // BGR
        int        keyline_thickness    = 1;
        double     point_cloud_opacity  = 1.0;
        float      point_size           = 4.0f;
    };

    // ========================================================================
    // Folder and File Paths
    // ========================================================================

    struct folder_options
    {
        std::filesystem::path root                 = ".";
        std::filesystem::path left                 = "cam0";
        std::filesystem::path right                = "cam1";
        std::filesystem::path output               = "output";
        std::filesystem::path calibration_file     = "camchain.yaml";
        std::filesystem::path groundtruth_file     = "groundtruth.csv";
        std::filesystem::path imu_calibration_file = "imu_config.yaml";
        std::filesystem::path imu_file             = "";
        double                timescale            = 1.0;
        size_t                max_frames           = 0; // 0 for unlimited
    };

    // ========================================================================
    // All Configuration Options (root-level)
    // ========================================================================

    struct all_options
    {
        // File and logging
        std::filesystem::path     file        = "options.yaml";
        spdlog::level::level_enum log_level   = spdlog::level::info;
        std::string               log_pattern = "[%H:%M:%S.%e] [%^%L%$] [%!:%s:%#] %v";
        verb                      verb_       = verb::RUN;

        // Configuration sections
        folder_options folder;
        gui_options    gui;
        slam_options   slam;
    };
} // namespace zenslam
