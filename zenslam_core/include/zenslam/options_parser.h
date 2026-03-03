#pragma once

#include <filesystem>

#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

#include "zenslam/all_options.h"

namespace zenslam
{
    /**
     * @brief Simple YAML parser for all_options and nested structures.
     *
     * Provides straightforward parse_* methods for each configuration struct.
     * No macros, no templates, no inheritance - just simple functions.
     */
    class options_parser
    {
    public:
        /// Load all options from a YAML file
        all_options load(const std::filesystem::path& yaml_file);

        /// Load all options from a YAML node
        all_options load_from_node(const YAML::Node& root);

    private:
        // ====================================================================
        // Individual struct parsers
        // ====================================================================

        detection_options     parse_detection(const YAML::Node& node);
        tracking_options      parse_tracking(const YAML::Node& node);
        triangulation_options parse_triangulation(const YAML::Node& node);
        keyframe_options      parse_keyframe(const YAML::Node& node);
        lba_options           parse_lba(const YAML::Node& node);
        pnp_options           parse_pnp(const YAML::Node& node);
        essential_options     parse_essential(const YAML::Node& node);
        rigid_options         parse_rigid(const YAML::Node& node);
        slam_options          parse_slam(const YAML::Node& node);
        gui_options           parse_gui(const YAML::Node& node);
        folder_options        parse_folder(const YAML::Node& node);

        // ====================================================================
        // Helper methods for type-safe YAML extraction
        // ====================================================================

        /// Get a value from YAML node, return default if missing or invalid
        template <typename T>
        static T get_or_default(const YAML::Node& node, const std::string& key, const T& default_val)
        {
            if (!node || !node[key]) return default_val;
            try
            {
                return node[key].as<T>();
            }
            catch (const std::exception&)
            {
                return default_val;
            }
        }

        /// Overload for std::filesystem::path
        static std::filesystem::path get_or_default
        (
            const YAML::Node&            node,
            const std::string&           key,
            const std::filesystem::path& default_val
        )
        {
            if (!node || !node[key]) return default_val;
            try
            {
                return { node[key].as<std::string>() };
            }
            catch (const std::exception&)
            {
                return default_val;
            }
        }

        /// Parse cv::Size from YAML array [width, height]
        static cv::Size get_size(const YAML::Node& node, const std::string& key, const cv::Size& default_val);

        /// Parse cv::Scalar from YAML array [v0, v1, v2, v3]
        cv::Scalar get_scalar(const YAML::Node& node, const std::string& key, const cv::Scalar& default_val);

        /// Parse enum from string using magic_enum
        template <typename E>
        static E get_enum(const YAML::Node& node, const std::string& key, const E& default_val)
        {
            if (!node || !node[key]) return default_val;
            try
            {
                auto str = node[key].as<std::string>();
                // This requires magic_enum - we'll implement in .cpp
                return default_val; // Placeholder
            }
            catch (const std::exception&)
            {
                return default_val;
            }
        }
    };
} // namespace zenslam
