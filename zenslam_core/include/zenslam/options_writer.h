#pragma once

#include <yaml-cpp/emitter.h>

#include "all_options.h"

namespace zenslam
{
    class options_writer
    {
    public:
        static void write(const all_options& opts);

    private:
        static auto write_detection(YAML::Emitter& emitter, const detection_options& detection_options) -> void;
        static auto write_triangulation(YAML::Emitter& emitter, const triangulation_options& triangulation_options) -> void;
        static auto write_keyframe(YAML::Emitter& emitter, const keyframe_options& keyframe_options) -> void;
        static auto write_lba(YAML::Emitter& emitter, const lba_options& lba_options) -> void;
        static auto write_pnp(YAML::Emitter& emitter, const pnp_options& pnp_options) -> void;
        static auto write_essential(YAML::Emitter& emitter, const essential_options& essential) -> void;
        static auto write_rigid(YAML::Emitter& emitter, const rigid_options& rigid_options) -> void;
        static auto write_slam(YAML::Emitter& emitter, const slam_options& slam_options) -> void;
    };
}
