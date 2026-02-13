#include "zenslam/keyframe_lba.h"

#include <unordered_map>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <ceres/rotation.h>

namespace zenslam
{
    namespace
    {
        struct ReprojectionError
        {
            ReprojectionError(const cv::Point3d& point_world,
                              const cv::Point2d& measurement,
                              const cv::Matx33d& K)
                : X(point_world)
                , uv(measurement)
                , fx(K(0, 0))
                , fy(K(1, 1))
                , cx(K(0, 2))
                , cy(K(1, 2))
            {
            }

            template <typename T>
            bool operator()(const T* const pose, T* residuals) const
            {
                const T Pw[3] = { T(X.x), T(X.y), T(X.z) };
                T Pc[3];
                ceres::QuaternionRotatePoint(pose, Pw, Pc);
                Pc[0] += pose[4];
                Pc[1] += pose[5];
                Pc[2] += pose[6];

                const T inv_z = T(1.0) / Pc[2];
                const T xp = Pc[0] * inv_z;
                const T yp = Pc[1] * inv_z;

                const T u = T(fx) * xp + T(cx);
                const T v = T(fy) * yp + T(cy);

                residuals[0] = u - T(uv.x);
                residuals[1] = v - T(uv.y);
                return true;
            }

            static auto Create(const cv::Point3d& point_world,
                               const cv::Point2d& measurement,
                               const cv::Matx33d& K) -> ceres::CostFunction*
            {
                return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7>(
                    new ReprojectionError(point_world, measurement, K));
            }

            cv::Point3d X;
            cv::Point2d uv;
            double fx = 0.0;
            double fy = 0.0;
            double cx = 0.0;
            double cy = 0.0;
        };

        auto affine_to_params(const cv::Affine3d& pose, double* params) -> void
        {
            const auto& R = pose.rotation();
            const double R_row_major[9] = {
                R(0, 0), R(0, 1), R(0, 2),
                R(1, 0), R(1, 1), R(1, 2),
                R(2, 0), R(2, 1), R(2, 2)
            };

            ceres::RotationMatrixToQuaternion(R_row_major, params);
            params[4] = pose.translation()[0];
            params[5] = pose.translation()[1];
            params[6] = pose.translation()[2];
        }

        auto params_to_affine(const double* params) -> cv::Affine3d
        {
            double R_row_major[9] = { };
            ceres::QuaternionToRotation(params, R_row_major);

            cv::Matx33d R(
                R_row_major[0], R_row_major[1], R_row_major[2],
                R_row_major[3], R_row_major[4], R_row_major[5],
                R_row_major[6], R_row_major[7], R_row_major[8]);

            const cv::Vec3d t(params[4], params[5], params[6]);
            return cv::Affine3d(R, t);
        }
    }

    keyframe_lba::keyframe_lba(calibration calib, lba_options options)
        : _calibration(std::move(calib))
        , _options(options)
    {
    }

    auto keyframe_lba::optimize(keyframe_database& db) -> keyframe_lba_result
    {
        keyframe_lba_result result{};
        const auto window = db.recent(_options.window_size);
        if (window.size() < 2)
        {
            return result;
        }

        const auto* anchor = window.front();
        if (!anchor)
        {
            return result;
        }

        std::vector<std::array<double, 7>> pose_params;
        pose_params.reserve(window.size());

        std::unordered_map<size_t, size_t> index_to_param;
        index_to_param.reserve(window.size());

        for (size_t i = 0; i < window.size(); ++i)
        {
            pose_params.emplace_back();
            affine_to_params(window[i]->pose, pose_params.back().data());
            index_to_param[window[i]->index] = i;
        }

        ceres::Problem problem;
        ceres::Manifold* pose_manifold = new ceres::ProductManifold(
            new ceres::QuaternionManifold(),
            new ceres::EuclideanManifold<3>());

        for (size_t i = 0; i < pose_params.size(); ++i)
        {
            problem.AddParameterBlock(pose_params[i].data(), 7, pose_manifold);
        }

        problem.SetParameterBlockConstant(pose_params[0].data());

        const auto& K = _calibration.camera_matrix[0];
        ceres::LossFunction* loss = _options.huber_threshold > 0.0
            ? new ceres::HuberLoss(_options.huber_threshold)
            : nullptr;

        size_t residuals = 0;
        for (const auto& [id, point] : anchor->points3d)
        {
            if (!anchor->keypoints[0].contains(id))
            {
                continue;
            }

            const cv::Point3d Xw = anchor->pose * point;

            for (const auto* frame : window)
            {
                if (!frame || !frame->keypoints[0].contains(id))
                {
                    continue;
                }

                const auto& kp = frame->keypoints[0].at(id);
                ceres::CostFunction* cost = ReprojectionError::Create(Xw, kp.pt, K);
                problem.AddResidualBlock(cost, loss, pose_params[index_to_param[frame->index]].data());
                residuals += 2;
            }
        }

        if (residuals == 0)
        {
            return result;
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = _options.max_iterations;
        options.num_threads = 1;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        for (const auto* frame : window)
        {
            const auto it = index_to_param.find(frame->index);
            if (it != index_to_param.end())
            {
                db.update_pose(frame->index, params_to_affine(pose_params[it->second].data()));
            }
        }

        result.converged = summary.termination_type == ceres::CONVERGENCE;
        result.keyframes = window.size();
        result.residuals = residuals;
        return result;
    }
}
