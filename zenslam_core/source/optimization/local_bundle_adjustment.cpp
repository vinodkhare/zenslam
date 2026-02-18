#include "zenslam/optimization/local_bundle_adjustment.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace
{
    struct local_observation
    {
        size_t      keyframe_id = 0;
        size_t      landmark_id = 0;
        cv::Point2d pixel       = { };
    };

    struct reprojection_error
    {
        reprojection_error(const cv::Point2d& observed_pixel, const cv::Matx33d& camera_matrix)
            : observed_x(observed_pixel.x)
            , observed_y(observed_pixel.y)
            , fx(camera_matrix(0, 0))
            , fy(camera_matrix(1, 1))
            , cx(camera_matrix(0, 2))
            , cy(camera_matrix(1, 2))
        {
        }

        template <typename T>
        auto operator()(const T* const pose_aa_t, const T* const point_xyz, T* residuals) const -> bool
        {
            const T* rotation = pose_aa_t;
            const T* translation = pose_aa_t + 3;

            T point_camera[3] = { };
            ceres::AngleAxisRotatePoint(rotation, point_xyz, point_camera);
            point_camera[0] += translation[0];
            point_camera[1] += translation[1];
            point_camera[2] += translation[2];

            const T depth = point_camera[2];
            const T safe_depth = depth > T(1e-9) ? depth : T(1e-9);

            const T x = point_camera[0] / safe_depth;
            const T y = point_camera[1] / safe_depth;

            const T u = T(fx) * x + T(cx);
            const T v = T(fy) * y + T(cy);

            residuals[0] = u - T(observed_x);
            residuals[1] = v - T(observed_y);

            return true;
        }

        double observed_x = 0.0;
        double observed_y = 0.0;
        double fx = 0.0;
        double fy = 0.0;
        double cx = 0.0;
        double cy = 0.0;
    };

    auto affine_to_pose_parameter(const cv::Affine3d& pose) -> std::array<double, 6>
    {
        const cv::Matx33d rotation_matrix = pose.rotation();
        cv::Mat rotation_cv(3, 3, CV_64F);

        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                rotation_cv.at<double>(r, c) = rotation_matrix(r, c);
            }
        }

        cv::Mat axis_angle_cv;
        cv::Rodrigues(rotation_cv, axis_angle_cv);

        const cv::Vec3d translation = pose.translation();
        return {
            axis_angle_cv.at<double>(0, 0),
            axis_angle_cv.at<double>(1, 0),
            axis_angle_cv.at<double>(2, 0),
            translation[0],
            translation[1],
            translation[2]
        };
    }

    auto pose_parameter_to_affine(const std::array<double, 6>& pose_aa_t) -> cv::Affine3d
    {
        cv::Mat axis_angle_cv(3, 1, CV_64F);
        axis_angle_cv.at<double>(0, 0) = pose_aa_t[0];
        axis_angle_cv.at<double>(1, 0) = pose_aa_t[1];
        axis_angle_cv.at<double>(2, 0) = pose_aa_t[2];

        cv::Mat rotation_cv;
        cv::Rodrigues(axis_angle_cv, rotation_cv);

        cv::Matx33d rotation_matrix;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                rotation_matrix(r, c) = rotation_cv.at<double>(r, c);
            }
        }

        const cv::Vec3d translation{ pose_aa_t[3], pose_aa_t[4], pose_aa_t[5] };
        return { rotation_matrix, translation };
    }

    auto reprojection_rmse(
        const std::vector<std::array<double, 6>>& keyframe_params,
        const std::vector<std::array<double, 3>>& landmark_params,
        const std::vector<local_observation>& observations,
        const std::unordered_map<size_t, size_t>& keyframe_index_by_id,
        const std::unordered_map<size_t, size_t>& landmark_index_by_id,
        const cv::Matx33d& camera_matrix) -> double
    {
        if (observations.empty())
        {
            return 0.0;
        }

        const double fx = camera_matrix(0, 0);
        const double fy = camera_matrix(1, 1);
        const double cx = camera_matrix(0, 2);
        const double cy = camera_matrix(1, 2);

        double squared_error_sum = 0.0;
        size_t residual_count = 0;

        for (const auto& obs : observations)
        {
            const auto keyframe_it = keyframe_index_by_id.find(obs.keyframe_id);
            const auto landmark_it = landmark_index_by_id.find(obs.landmark_id);
            if (keyframe_it == keyframe_index_by_id.end() || landmark_it == landmark_index_by_id.end())
            {
                continue;
            }

            const auto& pose = keyframe_params[keyframe_it->second];
            const auto& point = landmark_params[landmark_it->second];

            double point_camera[3] = { };
            ceres::AngleAxisRotatePoint(pose.data(), point.data(), point_camera);
            point_camera[0] += pose[3];
            point_camera[1] += pose[4];
            point_camera[2] += pose[5];

            if (point_camera[2] <= 1e-9)
            {
                continue;
            }

            const double u = fx * (point_camera[0] / point_camera[2]) + cx;
            const double v = fy * (point_camera[1] / point_camera[2]) + cy;

            const double du = u - obs.pixel.x;
            const double dv = v - obs.pixel.y;

            squared_error_sum += du * du + dv * dv;
            residual_count += 2;
        }

        if (residual_count == 0)
        {
            return 0.0;
        }

        return std::sqrt(squared_error_sum / static_cast<double>(residual_count));
    }
}

zenslam::local_bundle_adjustment::local_bundle_adjustment(cv::Matx33d camera_matrix, lba_options options)
    : _camera_matrix(camera_matrix)
    , _options(std::move(options))
{
}

auto zenslam::local_bundle_adjustment::optimize(
    std::unordered_map<size_t, frame::estimated>& keyframes,
    point3d_cloud& landmarks,
    const std::vector<size_t>& fixed_keyframe_ids) const -> result
{
    result optimize_result{ };

    if (keyframes.empty() || landmarks.empty())
    {
        optimize_result.converged = true;
        return optimize_result;
    }

    std::unordered_map<size_t, size_t> keyframe_index_by_id;
    keyframe_index_by_id.reserve(keyframes.size());
    std::vector<size_t> keyframe_ids;
    keyframe_ids.reserve(keyframes.size());
    std::vector<frame::estimated*> keyframe_ptrs;
    keyframe_ptrs.reserve(keyframes.size());

    for (const auto& [keyframe_id, _] : keyframes)
    {
        keyframe_ids.push_back(keyframe_id);
    }
    std::ranges::sort(keyframe_ids);

    for (size_t keyframe_param_index = 0; keyframe_param_index < keyframe_ids.size(); ++keyframe_param_index)
    {
        const size_t keyframe_id = keyframe_ids[keyframe_param_index];
        auto& keyframe = keyframes.at(keyframe_id);
        keyframe_index_by_id[keyframe_id] = keyframe_param_index;
        keyframe_ptrs.push_back(&keyframe);
    }

    std::unordered_map<size_t, size_t> landmark_index_by_id;
    landmark_index_by_id.reserve(landmarks.size());
    std::vector<size_t> landmark_ids;
    landmark_ids.reserve(landmarks.size());
    std::vector<point3d*> landmark_ptrs;
    landmark_ptrs.reserve(landmarks.size());

    for (const auto& [landmark_id, _] : landmarks)
    {
        landmark_ids.push_back(landmark_id);
    }
    std::ranges::sort(landmark_ids);

    for (size_t landmark_param_index = 0; landmark_param_index < landmark_ids.size(); ++landmark_param_index)
    {
        const size_t landmark_id = landmark_ids[landmark_param_index];
        auto& landmark = landmarks.at(landmark_id);
        landmark_index_by_id[landmark_id] = landmark_param_index;
        landmark_ptrs.push_back(&landmark);
    }

    std::vector<local_observation> observations;
    observations.reserve(keyframes.size() * 256);
    for (const auto& [keyframe_id, keyframe] : keyframes)
    {
        for (const auto& keypoint : keyframe.keypoints[0] | std::views::values)
        {
            if (!landmark_index_by_id.contains(keypoint.index))
            {
                continue;
            }

            if (!std::isfinite(keypoint.pt.x) || !std::isfinite(keypoint.pt.y))
            {
                continue;
            }

            const auto landmark_it = landmarks.find(keypoint.index);
            if (landmark_it == landmarks.end())
            {
                continue;
            }

            const cv::Point3d point_w{ landmark_it->second.x, landmark_it->second.y, landmark_it->second.z };
            const cv::Point3d point_c = keyframe.pose * point_w;
            if (!std::isfinite(point_c.z) || point_c.z <= 1e-3)
            {
                continue;
            }

            observations.push_back(
                local_observation{ keyframe_id, keypoint.index, cv::Point2d(keypoint.pt.x, keypoint.pt.y) });
        }
    }

    if (observations.empty())
    {
        optimize_result.converged = true;
        return optimize_result;
    }

    std::vector<std::array<double, 6>> keyframe_params;
    keyframe_params.reserve(keyframes.size());
    for (const auto* keyframe : keyframe_ptrs)
    {
        keyframe_params.push_back(affine_to_pose_parameter(keyframe->pose));
    }

    std::vector<std::array<double, 3>> landmark_params;
    landmark_params.reserve(landmarks.size());
    for (const auto* landmark : landmark_ptrs)
    {
        landmark_params.push_back({ landmark->x, landmark->y, landmark->z });
    }

    optimize_result.initial_rmse = reprojection_rmse(
        keyframe_params,
        landmark_params,
        observations,
        keyframe_index_by_id,
        landmark_index_by_id,
        _camera_matrix);

    ceres::Problem problem;

    for (const auto& observation : observations)
    {
        const auto keyframe_it = keyframe_index_by_id.find(observation.keyframe_id);
        const auto landmark_it = landmark_index_by_id.find(observation.landmark_id);
        if (keyframe_it == keyframe_index_by_id.end() || landmark_it == landmark_index_by_id.end())
        {
            continue;
        }

        auto* cost_function = new ceres::AutoDiffCostFunction<reprojection_error, 2, 6, 3>(
            new reprojection_error(observation.pixel, _camera_matrix));
        ceres::LossFunction* loss_function = nullptr;
        if (_options.refine_landmarks.value())
        {
            loss_function = new ceres::HuberLoss(_options.huber_delta.value());
        }

        problem.AddResidualBlock(
            cost_function,
            loss_function,
            keyframe_params[keyframe_it->second].data(),
            landmark_params[landmark_it->second].data());

        ++optimize_result.residuals;
    }

    const std::set<size_t> fixed_ids(fixed_keyframe_ids.begin(), fixed_keyframe_ids.end());

    if (fixed_ids.empty() && !keyframe_params.empty() && problem.HasParameterBlock(keyframe_params[0].data()))
    {
        problem.SetParameterBlockConstant(keyframe_params[0].data());
    }

    for (size_t i = 0; i < keyframe_ids.size(); ++i)
    {
        if (fixed_ids.contains(keyframe_ids[i]) && problem.HasParameterBlock(keyframe_params[i].data()))
        {
            problem.SetParameterBlockConstant(keyframe_params[i].data());
        }
    }

    if (!_options.refine_landmarks.value())
    {
        for (size_t i = 0; i < keyframe_ids.size(); ++i)
        {
            if (fixed_ids.contains(keyframe_ids[i]) || !problem.HasParameterBlock(keyframe_params[i].data()))
            {
                continue;
            }

            problem.SetManifold(keyframe_params[i].data(), new ceres::SubsetManifold(6, { 0, 1, 2 }));
        }

        for (size_t i = 0; i < landmark_params.size(); ++i)
        {
            if (problem.HasParameterBlock(landmark_params[i].data()))
            {
                problem.SetParameterBlockConstant(landmark_params[i].data());
            }
        }
    }

    ceres::Solver::Options solver_options;
    solver_options.max_num_iterations = std::max(1, _options.max_iterations.value());
    solver_options.num_threads = 1;
    solver_options.linear_solver_type = _options.refine_landmarks.value() ? ceres::SPARSE_SCHUR : ceres::DENSE_QR;
    solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    solver_options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    optimize_result.converged  = summary.termination_type == ceres::CONVERGENCE || summary.termination_type == ceres::USER_SUCCESS;
    optimize_result.iterations = static_cast<int>(summary.iterations.size());

    for (size_t i = 0; i < keyframe_ptrs.size(); ++i)
    {
        keyframe_ptrs[i]->pose = pose_parameter_to_affine(keyframe_params[i]);
    }

    for (size_t i = 0; i < landmark_ptrs.size(); ++i)
    {
        landmark_ptrs[i]->x = landmark_params[i][0];
        landmark_ptrs[i]->y = landmark_params[i][1];
        landmark_ptrs[i]->z = landmark_params[i][2];
    }

    optimize_result.final_rmse = reprojection_rmse(
        keyframe_params,
        landmark_params,
        observations,
        keyframe_index_by_id,
        landmark_index_by_id,
        _camera_matrix);

    return optimize_result;
}
