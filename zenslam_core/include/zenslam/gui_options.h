#pragma once

#include <opencv2/core/types.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

#include "zenslam/options_base.h"

namespace zenslam
{
    /// GUI and visualization configuration
    class gui_options : public options_base<gui_options, "gui options", "gui">
    {
    public:
        // Inherited from options_base:
        // static constexpr auto name() - returns "gui options"
        // static constexpr auto prefix() - returns "gui"
        // static auto description() -> boost::program_options::options_description;
        // static auto parse_yaml(const YAML::Node& node) -> gui_options;
        // static void parse_cli(gui_options& options, const std::map<std::string, boost::program_options::basic_option<char>>& options_map, const boost::program_options::variables_map& vm);
        // void print() const;

        ZENSLAM_DEFINE_OPTIONS(
            ((bool, show_keypoints, true, "Show keypoints in visualization"))
            ((bool, show_keylines, true, "Show keylines in visualization"))
            ((cv::Scalar, keyline_single_color, cv::Scalar(0, 255, 0), "Keyline single color (BGR)"))
            ((cv::Scalar, keyline_match_color, cv::Scalar(0, 0, 255), "Keyline match color (BGR)"))
            ((int, keyline_thickness, 1, "Keyline line thickness in pixels"))
            ((float, point_cloud_opacity, 1.0f, "Point cloud opacity (0.0-1.0)"))
            ((float, point_size, 4.0f, "Point cloud point size (1.0-20.0)"))
        )

        void validate() const;
    };
} // namespace zenslam
