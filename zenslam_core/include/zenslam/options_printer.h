#pragma once

#include "zenslam/all_options.h"

namespace zenslam
{
    /**
     * @brief Simple printer for all_options structures.
     *
     * Logs all configuration to spdlog for debugging and verification.
     */
    class options_printer {
    public:
        /// Print all options to log
        static void print(const all_options& opts);

    private:
        static void print_detection(const detection_options& opts);
        static void print_tracking(const tracking_options& opts);
        static void print_triangulation(const triangulation_options& opts);
        static void print_keyframe(const keyframe_options& opts);
        static void print_lba(const lba_options& opts);
        static void print_pnp(const pnp_options& opts);
        static void print_essential(const essential_options& opts);
        static void print_rigid(const rigid_options& opts);
        static void print_slam(const slam_options& opts);
        static void print_gui(const gui_options& opts);
        static void print_folder(const folder_options& opts);
    };

}  // namespace zenslam
