#pragma once

#include <thread>

#include <opencv2/core/affine.hpp>

#include "event.h"
#include "options.h"
#include "stereo_folder_reader.h"
#include "stereo_frame.h"

namespace zenslam
{
    class slam_thread
    {
    public:
        event<stereo_frame> on_frame;

        explicit slam_thread(options options);
        ~slam_thread();

        static void track
        (
            const stereo_frame &frame_0,
            stereo_frame &      frame_1
        );

        static void correspondences
        (
            const stereo_frame &      frame_0,
            const stereo_frame &      frame_1,
            std::vector<cv::Point3d> &points3d,
            std::vector<cv::Point2d> &points2d
        );

        static void solve_pnp
        (
            const cv::Matx33d &             camera_matrix,
            const std::vector<cv::Point3d> &points3d,
            const std::vector<cv::Point2d> &points2d,
            cv::Affine3d &                  pose
        );

    private:
        options _options { };

        std::stop_source _stop_source { };
        std::stop_token  _stop_token { _stop_source.get_token() };
        std::jthread     _thread { &slam_thread::loop, this };

        void loop();
    };
}
