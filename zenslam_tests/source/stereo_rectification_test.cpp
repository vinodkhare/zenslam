#include <catch2/catch_all.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <random>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include <zenslam/frame/estimated.h>
#include <zenslam/local_bundle_adjustment.h>
#include <zenslam/options.h>
#include <zenslam/types/point3d_cloud.h>
#include <zenslam/types/point3d.h>
#include <zenslam/utils.h>
#include <zenslam/utils_opencv.h>

namespace
{
    struct bal_observation
    {
        size_t camera_id = 0;
        size_t point_id = 0;
        double x = 0.0;
        double y = 0.0;
    };

    struct bal_problem_data
    {
        size_t num_cameras = 0;
        size_t num_points = 0;
        size_t num_observations = 0;
        std::vector<bal_observation> observations;
        std::vector<std::array<double, 9>> cameras;
        std::vector<cv::Point3d> points;
    };

    auto find_bal_file(const std::string& filename) -> std::filesystem::path
    {
        const std::vector<std::filesystem::path> candidates = {
            std::filesystem::path("zenslam_tests/data/bal") / filename,
            std::filesystem::path("../zenslam_tests/data/bal") / filename,
            std::filesystem::path("../../zenslam_tests/data/bal") / filename,
            std::filesystem::path("../../../zenslam_tests/data/bal") / filename
        };

        for (const auto& candidate : candidates)
        {
            if (std::filesystem::exists(candidate))
            {
                return candidate;
            }
        }

        return {};
    }

    auto load_bal_problem(const std::filesystem::path& file_path) -> bal_problem_data
    {
        std::ifstream input(file_path);
        if (!input)
        {
            throw std::runtime_error("Failed to open BAL file");
        }

        bal_problem_data bal;
        input >> bal.num_cameras >> bal.num_points >> bal.num_observations;
        if (!input)
        {
            throw std::runtime_error("Invalid BAL header");
        }

        bal.observations.resize(bal.num_observations);
        for (size_t i = 0; i < bal.num_observations; ++i)
        {
            input >> bal.observations[i].camera_id >> bal.observations[i].point_id >> bal.observations[i].x >> bal.observations[i].y;
        }

        bal.cameras.resize(bal.num_cameras);
        for (size_t camera_idx = 0; camera_idx < bal.num_cameras; ++camera_idx)
        {
            for (size_t param_idx = 0; param_idx < 9; ++param_idx)
            {
                input >> bal.cameras[camera_idx][param_idx];
            }
        }

        bal.points.resize(bal.num_points);
        for (size_t point_idx = 0; point_idx < bal.num_points; ++point_idx)
        {
            input >> bal.points[point_idx].x >> bal.points[point_idx].y >> bal.points[point_idx].z;
        }

        if (!input)
        {
            throw std::runtime_error("Invalid BAL payload");
        }

        return bal;
    }

    auto camera_pose_from_bal(const std::array<double, 9>& camera) -> cv::Affine3d
    {
        cv::Mat axis_angle(3, 1, CV_64F);
        axis_angle.at<double>(0, 0) = camera[0];
        axis_angle.at<double>(1, 0) = camera[1];
        axis_angle.at<double>(2, 0) = camera[2];

        cv::Mat rotation_cv;
        cv::Rodrigues(axis_angle, rotation_cv);

        const cv::Matx33d rotation(rotation_cv);
        const cv::Vec3d translation{ camera[3], camera[4], camera[5] };
        return { rotation, translation };
    }

    auto run_bal_subset_lba_case(const std::filesystem::path& bal_path) -> void
    {
        const auto bal = load_bal_problem(bal_path);

        constexpr size_t max_cameras = 8;
        constexpr size_t max_points = 600;
        constexpr size_t max_observations = 3000;

        std::vector<bal_observation> selected_observations;
        selected_observations.reserve(max_observations);

        for (const auto& observation : bal.observations)
        {
            if (observation.camera_id >= max_cameras || observation.point_id >= max_points)
            {
                continue;
            }

            selected_observations.push_back(observation);
            if (selected_observations.size() >= max_observations)
            {
                break;
            }
        }

        std::unordered_map<size_t, size_t> camera_observation_count;
        std::unordered_map<size_t, size_t> point_observation_count;
        for (const auto& observation : selected_observations)
        {
            ++camera_observation_count[observation.camera_id];
            ++point_observation_count[observation.point_id];
        }

        std::vector<bal_observation> filtered_observations;
        filtered_observations.reserve(selected_observations.size());
        for (const auto& observation : selected_observations)
        {
            if (camera_observation_count[observation.camera_id] < 150)
            {
                continue;
            }

            if (point_observation_count[observation.point_id] < 3)
            {
                continue;
            }

            filtered_observations.push_back(observation);
        }

        selected_observations = std::move(filtered_observations);

        REQUIRE(selected_observations.size() > 1000);

        std::set<size_t> used_camera_ids;
        std::set<size_t> used_point_ids;
        for (const auto& observation : selected_observations)
        {
            used_camera_ids.insert(observation.camera_id);
            used_point_ids.insert(observation.point_id);
        }

        REQUIRE(used_camera_ids.size() >= 2);

        const cv::Matx33d k = {
            500.0, 0.0, 320.0,
            0.0, 500.0, 240.0,
            0.0, 0.0, 1.0
        };

        std::unordered_map<size_t, cv::Affine3d> gt_poses;
        std::unordered_map<size_t, zenslam::frame::estimated> keyframes_init;
        keyframes_init.reserve(used_camera_ids.size());

        std::mt19937 rng(97531);
        std::normal_distribution<double> pose_noise(0.0, 0.02);

        for (const auto camera_id : used_camera_ids)
        {
            const auto pose_gt = cv::Affine3d(
                cv::Matx33d::eye(),
                cv::Vec3d(0.10 * static_cast<double>(camera_id), 0.01 * static_cast<double>(camera_id), 0.0));
            gt_poses[camera_id] = pose_gt;

            zenslam::frame::estimated keyframe;
            keyframe.index = camera_id;
            keyframe.pose = cv::Affine3d(
                pose_gt.rotation(),
                cv::Vec3d(
                    pose_gt.translation()[0] + pose_noise(rng),
                    pose_gt.translation()[1] + pose_noise(rng),
                    pose_gt.translation()[2] + pose_noise(rng)));

            keyframes_init.emplace(camera_id, std::move(keyframe));
        }

        zenslam::point3d_cloud landmarks_init;
        std::unordered_map<size_t, cv::Point3d> landmarks_gt;
        for (const auto point_id : used_point_ids)
        {
            const auto& bal_point = bal.points[point_id];
            const cv::Point3d point_gt{
                bal_point.x * 0.02,
                bal_point.y * 0.02,
                4.0 + std::abs(bal_point.z) * 0.02
            };
            landmarks_gt[point_id] = point_gt;

            zenslam::point3d point_init;
            point_init.index = point_id;
            point_init.x = point_gt.x;
            point_init.y = point_gt.y;
            point_init.z = point_gt.z;
            landmarks_init += point_init;
        }

        for (const auto& observation : selected_observations)
        {
            const auto keyframe_it = keyframes_init.find(observation.camera_id);
            const auto point_it = landmarks_gt.find(observation.point_id);
            if (keyframe_it == keyframes_init.end() || point_it == landmarks_gt.end())
            {
                continue;
            }

            const auto pixel = zenslam::utils::project_point(k, gt_poses.at(observation.camera_id), point_it->second);

            zenslam::keypoint keypoint;
            keypoint.index = observation.point_id;
            keypoint.pt = cv::Point2f(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
            keyframe_it->second.keypoints[0] += keypoint;
        }

        size_t expected_residuals = 0;
        for (const auto& [_, keyframe] : keyframes_init)
        {
            expected_residuals += keyframe.keypoints[0].size();
        }

        const auto fixed_keyframe_id = *used_camera_ids.begin();
        const auto second_fixed_keyframe_id = *std::next(used_camera_ids.begin());

        double translation_error_before = 0.0;
        for (const auto& [camera_id, keyframe] : keyframes_init)
        {
            if (camera_id == fixed_keyframe_id || camera_id == second_fixed_keyframe_id)
            {
                continue;
            }

            translation_error_before += cv::norm(keyframe.pose.translation() - gt_poses.at(camera_id).translation());
        }

        zenslam::lba_options ba_options;
        ba_options.max_iterations = 80;
        ba_options.huber_delta = 1.0;
        ba_options.refine_landmarks = false;

        zenslam::local_bundle_adjustment lba(k, ba_options);
        const auto result = lba.optimize(keyframes_init, landmarks_init, { fixed_keyframe_id, second_fixed_keyframe_id });

        double translation_error_after = 0.0;
        for (const auto& [camera_id, keyframe] : keyframes_init)
        {
            if (camera_id == fixed_keyframe_id || camera_id == second_fixed_keyframe_id)
            {
                continue;
            }

            translation_error_after += cv::norm(keyframe.pose.translation() - gt_poses.at(camera_id).translation());
        }

        REQUIRE(expected_residuals > 1000);
        REQUIRE(result.residuals == expected_residuals);
        REQUIRE(result.final_rmse < result.initial_rmse);
        REQUIRE(translation_error_after < translation_error_before);
    }
}

TEST_CASE("Stereo rectification option", "[options][stereo_rectify]")
{
    using zenslam::options;

    SECTION("Default stereo_rectify is false")
    {
        const auto opts = options{};
        REQUIRE(opts.slam->stereo_rectify == false);
    }

    SECTION("stereo_rectify can be set to true")
    {
        auto opts = options{};
        opts.slam->stereo_rectify = true;
        REQUIRE(opts.slam->stereo_rectify == true);
        REQUIRE_NOTHROW(opts.slam->validate());
    }
}

TEST_CASE("utils::rectify function", "[utils][stereo_rectify]")
{
    SECTION("rectify handles empty image gracefully")
    {
        cv::Mat empty_image;
        cv::Mat map_x, map_y;
        
        // Should not crash with empty maps
        auto result = zenslam::utils::rectify(empty_image, map_x, map_y);
        REQUIRE(result.empty());
    }

    SECTION("rectify with identity maps produces similar image")
    {
        // Create a simple test image
        cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::circle(test_image, cv::Point(320, 240), 50, cv::Scalar(255), -1);
        
        // Create identity maps (no transformation)
        cv::Mat map_x(480, 640, CV_32FC1);
        cv::Mat map_y(480, 640, CV_32FC1);
        for (int y = 0; y < 480; y++)
        {
            for (int x = 0; x < 640; x++)
            {
                map_x.at<float>(y, x) = static_cast<float>(x);
                map_y.at<float>(y, x) = static_cast<float>(y);
            }
        }
        
        auto result = zenslam::utils::rectify(test_image, map_x, map_y);
        
        // Result should have the same size as input
        REQUIRE(result.rows == test_image.rows);
        REQUIRE(result.cols == test_image.cols);
        REQUIRE(result.type() == test_image.type());
        
        // With identity maps, the image should be very similar
        // (may have minor interpolation differences)
        cv::Mat diff;
        cv::absdiff(test_image, result, diff);
        double max_diff = 0;
        cv::minMaxLoc(diff, nullptr, &max_diff);
        
        // Allow small differences due to interpolation
        REQUIRE(max_diff < 5.0);
    }
}

TEST_CASE("local bundle adjustment optimizes keyframe poses", "[optimization][lba]")
{
    const cv::Matx33d k = {
        500.0, 0.0, 320.0,
        0.0, 500.0, 240.0,
        0.0, 0.0, 1.0
    };

    const cv::Affine3d pose0_gt = cv::Affine3d::Identity();
    const cv::Affine3d pose1_gt = cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.20, 0.02, 0.00));

    std::vector<zenslam::point3d> landmarks_gt;
    zenslam::point3d_cloud landmarks_init;

    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> x_dist(-0.6, 0.6);
    std::uniform_real_distribution<double> y_dist(-0.4, 0.4);
    std::uniform_real_distribution<double> z_dist(3.0, 5.0);
    std::normal_distribution<double> noise(0.0, 0.03);

    constexpr size_t landmark_count = 30;
    landmarks_gt.reserve(landmark_count);

    for (size_t i = 0; i < landmark_count; ++i)
    {
        const cv::Point3d point_gt{ x_dist(rng), y_dist(rng), z_dist(rng) };

        zenslam::point3d landmark_gt;
        landmark_gt.index = i;
        landmark_gt.x = point_gt.x;
        landmark_gt.y = point_gt.y;
        landmark_gt.z = point_gt.z;
        landmarks_gt.push_back(landmark_gt);

        zenslam::point3d landmark_init;
        landmark_init.index = i;
        landmark_init.x = point_gt.x + noise(rng);
        landmark_init.y = point_gt.y + noise(rng);
        landmark_init.z = point_gt.z + noise(rng);
        landmarks_init += landmark_init;
    }

    zenslam::frame::estimated keyframe0;
    keyframe0.index = 0;
    keyframe0.pose = pose0_gt;

    zenslam::frame::estimated keyframe1;
    keyframe1.index = 1;
    keyframe1.pose = cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.33, -0.08, 0.05));

    for (const auto& lm : landmarks_gt)
    {
        const cv::Point3d point_w{ lm.x, lm.y, lm.z };
        const auto pixel0 = zenslam::utils::project_point(k, pose0_gt, point_w);
        const auto pixel1 = zenslam::utils::project_point(k, pose1_gt, point_w);

        zenslam::keypoint kp0;
        kp0.index = lm.index;
        kp0.pt = cv::Point2f(static_cast<float>(pixel0.x), static_cast<float>(pixel0.y));
        keyframe0.keypoints[0] += kp0;

        zenslam::keypoint kp1;
        kp1.index = lm.index;
        kp1.pt = cv::Point2f(static_cast<float>(pixel1.x), static_cast<float>(pixel1.y));
        keyframe1.keypoints[0] += kp1;
    }

    std::unordered_map<size_t, zenslam::frame::estimated> keyframes_init = {
        { keyframe0.index, keyframe0 },
        { keyframe1.index, keyframe1 }
    };

    const double translation_error_before = cv::norm(keyframes_init.at(1).pose.translation() - pose1_gt.translation());

    zenslam::lba_options ba_options;
    ba_options.max_iterations = 50;
    ba_options.huber_delta = 1.0;
    ba_options.refine_landmarks = true;

    zenslam::local_bundle_adjustment lba(k, ba_options);
    const auto result = lba.optimize(keyframes_init, landmarks_init, { 0 });

    const double translation_error_after = cv::norm(keyframes_init.at(1).pose.translation() - pose1_gt.translation());

    REQUIRE(result.residuals == landmark_count * 2);
    REQUIRE(result.converged);
    REQUIRE(result.final_rmse < result.initial_rmse);
    REQUIRE(translation_error_after < translation_error_before);
    REQUIRE(translation_error_after < 0.03);
}

TEST_CASE("local bundle adjustment handles larger keyframe windows", "[optimization][lba]")
{
    const cv::Matx33d k = {
        520.0, 0.0, 320.0,
        0.0, 520.0, 240.0,
        0.0, 0.0, 1.0
    };

    constexpr size_t keyframe_count = 6;
    constexpr size_t landmark_count = 80;

    std::mt19937 rng(56789);
    std::uniform_real_distribution<double> x_dist(-0.9, 0.9);
    std::uniform_real_distribution<double> y_dist(-0.5, 0.5);
    std::uniform_real_distribution<double> z_dist(3.5, 6.0);
    std::normal_distribution<double> landmark_noise(0.0, 0.04);
    std::normal_distribution<double> pose_noise(0.0, 0.05);

    std::vector<zenslam::point3d> landmarks_gt;
    landmarks_gt.reserve(landmark_count);

    zenslam::point3d_cloud landmarks_init;

    for (size_t i = 0; i < landmark_count; ++i)
    {
        const cv::Point3d point_gt{ x_dist(rng), y_dist(rng), z_dist(rng) };

        zenslam::point3d lm_gt;
        lm_gt.index = i;
        lm_gt.x = point_gt.x;
        lm_gt.y = point_gt.y;
        lm_gt.z = point_gt.z;
        landmarks_gt.push_back(lm_gt);

        zenslam::point3d lm_init;
        lm_init.index = i;
        lm_init.x = point_gt.x + landmark_noise(rng);
        lm_init.y = point_gt.y + landmark_noise(rng);
        lm_init.z = point_gt.z + landmark_noise(rng);
        landmarks_init += lm_init;
    }

    std::vector<cv::Affine3d> gt_poses;
    gt_poses.reserve(keyframe_count);
    for (size_t i = 0; i < keyframe_count; ++i)
    {
        gt_poses.emplace_back(cv::Matx33d::eye(), cv::Vec3d(0.12 * static_cast<double>(i), 0.01 * static_cast<double>(i), 0.0));
    }

    std::unordered_map<size_t, zenslam::frame::estimated> keyframes_init;
    keyframes_init.reserve(keyframe_count);

    for (size_t i = 0; i < keyframe_count; ++i)
    {
        zenslam::frame::estimated kf;
        kf.index = i;
        kf.pose = cv::Affine3d(
            cv::Matx33d::eye(),
            cv::Vec3d(
                gt_poses[i].translation()[0] + pose_noise(rng),
                gt_poses[i].translation()[1] + pose_noise(rng),
                gt_poses[i].translation()[2] + pose_noise(rng)));

        for (const auto& lm : landmarks_gt)
        {
            const cv::Point3d point_w{ lm.x, lm.y, lm.z };
            const auto pixel = zenslam::utils::project_point(k, gt_poses[i], point_w);

            zenslam::keypoint kp;
            kp.index = lm.index;
            kp.pt = cv::Point2f(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
            kf.keypoints[0] += kp;
        }

        keyframes_init.emplace(i, std::move(kf));
    }

    double translation_error_before = 0.0;
    for (size_t i = 1; i < keyframe_count; ++i)
    {
        translation_error_before += cv::norm(keyframes_init.at(i).pose.translation() - gt_poses[i].translation());
    }

    zenslam::lba_options ba_options;
    ba_options.max_iterations = 80;
    ba_options.huber_delta = 1.0;
    ba_options.refine_landmarks = true;

    zenslam::local_bundle_adjustment lba(k, ba_options);
    const auto result = lba.optimize(keyframes_init, landmarks_init, { 0 });

    double translation_error_after = 0.0;
    for (size_t i = 1; i < keyframe_count; ++i)
    {
        translation_error_after += cv::norm(keyframes_init.at(i).pose.translation() - gt_poses[i].translation());
    }

    REQUIRE(result.residuals == keyframe_count * landmark_count);
    REQUIRE(result.converged);
    REQUIRE(result.final_rmse < result.initial_rmse);
    REQUIRE(translation_error_after < translation_error_before);
}

TEST_CASE("local bundle adjustment handles larger landmark sets", "[optimization][lba]")
{
    const cv::Matx33d k = {
        500.0, 0.0, 320.0,
        0.0, 500.0, 240.0,
        0.0, 0.0, 1.0
    };

    constexpr size_t landmark_count = 250;

    const cv::Affine3d pose0_gt = cv::Affine3d::Identity();
    const cv::Affine3d pose1_gt = cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.24, -0.01, 0.02));

    std::vector<zenslam::point3d> landmarks_gt;
    landmarks_gt.reserve(landmark_count);
    zenslam::point3d_cloud landmarks_init;

    std::mt19937 rng(24680);
    std::uniform_real_distribution<double> x_dist(-1.2, 1.2);
    std::uniform_real_distribution<double> y_dist(-0.8, 0.8);
    std::uniform_real_distribution<double> z_dist(3.0, 7.5);
    std::normal_distribution<double> noise(0.0, 0.02);

    for (size_t i = 0; i < landmark_count; ++i)
    {
        const cv::Point3d point_gt{ x_dist(rng), y_dist(rng), z_dist(rng) };

        zenslam::point3d landmark_gt;
        landmark_gt.index = i;
        landmark_gt.x = point_gt.x;
        landmark_gt.y = point_gt.y;
        landmark_gt.z = point_gt.z;
        landmarks_gt.push_back(landmark_gt);

        zenslam::point3d landmark_init;
        landmark_init.index = i;
        landmark_init.x = point_gt.x + noise(rng);
        landmark_init.y = point_gt.y + noise(rng);
        landmark_init.z = point_gt.z + noise(rng);
        landmarks_init += landmark_init;
    }

    zenslam::frame::estimated keyframe0;
    keyframe0.index = 0;
    keyframe0.pose = pose0_gt;

    zenslam::frame::estimated keyframe1;
    keyframe1.index = 1;
    keyframe1.pose = cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.36, -0.10, 0.08));

    for (const auto& lm : landmarks_gt)
    {
        const cv::Point3d point_w{ lm.x, lm.y, lm.z };
        const auto pixel0 = zenslam::utils::project_point(k, pose0_gt, point_w);
        const auto pixel1 = zenslam::utils::project_point(k, pose1_gt, point_w);

        zenslam::keypoint kp0;
        kp0.index = lm.index;
        kp0.pt = cv::Point2f(static_cast<float>(pixel0.x), static_cast<float>(pixel0.y));
        keyframe0.keypoints[0] += kp0;

        zenslam::keypoint kp1;
        kp1.index = lm.index;
        kp1.pt = cv::Point2f(static_cast<float>(pixel1.x), static_cast<float>(pixel1.y));
        keyframe1.keypoints[0] += kp1;
    }

    std::unordered_map<size_t, zenslam::frame::estimated> keyframes_init = {
        { keyframe0.index, keyframe0 },
        { keyframe1.index, keyframe1 }
    };

    const double translation_error_before = cv::norm(keyframes_init.at(1).pose.translation() - pose1_gt.translation());

    zenslam::lba_options ba_options;
    ba_options.max_iterations = 70;
    ba_options.huber_delta = 1.0;
    ba_options.refine_landmarks = true;

    zenslam::local_bundle_adjustment lba(k, ba_options);
    const auto result = lba.optimize(keyframes_init, landmarks_init, { 0 });

    const double translation_error_after = cv::norm(keyframes_init.at(1).pose.translation() - pose1_gt.translation());

    REQUIRE(result.residuals == landmark_count * 2);
    REQUIRE(result.converged);
    REQUIRE(result.final_rmse < result.initial_rmse);
    REQUIRE(translation_error_after < translation_error_before);
    REQUIRE(translation_error_after < 0.05);
}

TEST_CASE("local bundle adjustment can keep landmarks fixed", "[optimization][lba]")
{
    const cv::Matx33d k = {
        500.0, 0.0, 320.0,
        0.0, 500.0, 240.0,
        0.0, 0.0, 1.0
    };

    const cv::Affine3d pose0_gt = cv::Affine3d::Identity();
    const cv::Affine3d pose1_gt = cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.18, 0.03, 0.01));

    constexpr size_t landmark_count = 120;

    std::vector<zenslam::point3d> landmarks_gt;
    landmarks_gt.reserve(landmark_count);
    zenslam::point3d_cloud landmarks_init;

    std::mt19937 rng(13579);
    std::uniform_real_distribution<double> x_dist(-0.8, 0.8);
    std::uniform_real_distribution<double> y_dist(-0.6, 0.6);
    std::uniform_real_distribution<double> z_dist(3.5, 6.5);
    std::normal_distribution<double> noise(0.0, 0.03);

    for (size_t i = 0; i < landmark_count; ++i)
    {
        const cv::Point3d point_gt{ x_dist(rng), y_dist(rng), z_dist(rng) };

        zenslam::point3d landmark_gt;
        landmark_gt.index = i;
        landmark_gt.x = point_gt.x;
        landmark_gt.y = point_gt.y;
        landmark_gt.z = point_gt.z;
        landmarks_gt.push_back(landmark_gt);

        zenslam::point3d landmark_init;
        landmark_init.index = i;
        landmark_init.x = point_gt.x + noise(rng);
        landmark_init.y = point_gt.y + noise(rng);
        landmark_init.z = point_gt.z + noise(rng);
        landmarks_init += landmark_init;
    }

    const auto landmarks_before = landmarks_init;

    zenslam::frame::estimated keyframe0;
    keyframe0.index = 0;
    keyframe0.pose = pose0_gt;

    zenslam::frame::estimated keyframe1;
    keyframe1.index = 1;
    keyframe1.pose = cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.30, -0.06, 0.05));

    for (const auto& lm : landmarks_gt)
    {
        const cv::Point3d point_w{ lm.x, lm.y, lm.z };
        const auto pixel0 = zenslam::utils::project_point(k, pose0_gt, point_w);
        const auto pixel1 = zenslam::utils::project_point(k, pose1_gt, point_w);

        zenslam::keypoint kp0;
        kp0.index = lm.index;
        kp0.pt = cv::Point2f(static_cast<float>(pixel0.x), static_cast<float>(pixel0.y));
        keyframe0.keypoints[0] += kp0;

        zenslam::keypoint kp1;
        kp1.index = lm.index;
        kp1.pt = cv::Point2f(static_cast<float>(pixel1.x), static_cast<float>(pixel1.y));
        keyframe1.keypoints[0] += kp1;
    }

    std::unordered_map<size_t, zenslam::frame::estimated> keyframes_init = {
        { keyframe0.index, keyframe0 },
        { keyframe1.index, keyframe1 }
    };

    const double translation_error_before = cv::norm(keyframes_init.at(1).pose.translation() - pose1_gt.translation());

    zenslam::lba_options ba_options;
    ba_options.max_iterations = 60;
    ba_options.huber_delta = 1.0;
    ba_options.refine_landmarks = false;

    zenslam::local_bundle_adjustment lba(k, ba_options);
    const auto result = lba.optimize(keyframes_init, landmarks_init, { 0 });

    const double translation_error_after = cv::norm(keyframes_init.at(1).pose.translation() - pose1_gt.translation());

    REQUIRE(result.residuals == landmark_count * 2);
    REQUIRE(result.converged);
    REQUIRE(result.final_rmse < result.initial_rmse);
    REQUIRE(translation_error_after < translation_error_before);

    for (const auto& [landmark_id, landmark_before] : landmarks_before)
    {
        REQUIRE(landmarks_init.find(landmark_id) != landmarks_init.end());
        const auto& landmark_after = landmarks_init.at(landmark_id);
        REQUIRE(landmark_after.x == Catch::Approx(landmark_before.x).margin(1e-12));
        REQUIRE(landmark_after.y == Catch::Approx(landmark_before.y).margin(1e-12));
        REQUIRE(landmark_after.z == Catch::Approx(landmark_before.z).margin(1e-12));
    }
}

TEST_CASE("local bundle adjustment runs on BAL dataset subset", "[optimization][lba][bal]")
{
    const auto bal_path = find_bal_file("problem-49-7776-pre.txt");
    if (bal_path.empty())
    {
        SKIP("BAL file not found. Expected zenslam_tests/data/bal/problem-49-7776-pre.txt");
    }
    run_bal_subset_lba_case(bal_path);
}

TEST_CASE("local bundle adjustment runs on BAL Trafalgar subset", "[optimization][lba][bal]")
{
    const auto bal_path = find_bal_file("problem-21-11315-pre.txt");
    if (bal_path.empty())
    {
        SKIP("BAL file not found. Expected zenslam_tests/data/bal/problem-21-11315-pre.txt");
    }

    run_bal_subset_lba_case(bal_path);
}

TEST_CASE("local bundle adjustment runs on BAL Dubrovnik subset", "[optimization][lba][bal]")
{
    const auto bal_path = find_bal_file("problem-16-22106-pre.txt");
    if (bal_path.empty())
    {
        SKIP("BAL file not found. Expected zenslam_tests/data/bal/problem-16-22106-pre.txt");
    }

    run_bal_subset_lba_case(bal_path);
}
