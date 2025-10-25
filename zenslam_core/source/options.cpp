#include "options.h"

#include <print>

#include <boost/program_options.hpp>

#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "utils.h"

#include <filesystem>

namespace
{
    // Clamp an integer to [0, 255]
    int clamp_color(const int v)
    {
        return std::max(0, std::min(255, v));
    }

    void set_color_from_vec(const std::vector<int>& v, cv::Scalar& out)
    {
        if (v.size() == 3)
        {
            out = cv::Scalar(clamp_color(v[0]), clamp_color(v[1]), clamp_color(v[2]));
        }
    }

    // CLI helpers
    template <class T>
    void set_if_provided
    (
        const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
        const boost::program_options::variables_map&                             vm,
        const std::string_view                                                   key,
        T&                                                                       out
    )
    {
        const auto k  = std::string(key);
        const auto it = options_map.find(k);
        if (it != options_map.end())
        {
            const auto vit = vm.find(k);
            if (vit != vm.end()) out = vit->second.as<T>();
        }
    }

    void set_color_if_provided
    (
        const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
        const boost::program_options::variables_map&                             vm,
        const std::string_view                                                   key,
        cv::Scalar&                                                              out
    )
    {
        const auto k  = std::string(key);
        const auto it = options_map.find(k);
        if (it != options_map.end())
        {
            const auto vit = vm.find(k);
            if (vit != vm.end()) set_color_from_vec(vit->second.as<std::vector<int>>(), out);
        }
    }

    template <class E>
    void set_enum_if_provided
    (
        const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
        const boost::program_options::variables_map&                             vm,
        const std::string_view                                                   key,
        E&                                                                       out
    )
    {
        const auto k  = std::string(key);
        const auto it = options_map.find(k);
        if (it != options_map.end())
        {
            const auto vit = vm.find(k);
            if (vit != vm.end())
            {
                if (auto v = magic_enum::enum_cast<E>(vit->second.as<std::string>())) out = *v;
            }
        }
    }

    void set_path_if_provided
    (
        const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
        const boost::program_options::variables_map&                             vm,
        const std::string_view                                                   key,
        std::filesystem::path&                                                   out
    )
    {
        const auto k  = std::string(key);
        const auto it = options_map.find(k);
        if (it != options_map.end())
        {
            const auto vit = vm.find(k);
            if (vit != vm.end()) out = vit->second.as<std::string>();
        }
    }

    // YAML helpers
    template <class T>
    void yaml_set_if_present(const YAML::Node& node, const std::string_view key, T& out)
    {
        if (const auto n = node[std::string(key)]) out = n.as<T>();
    }

    void yaml_set_path(const YAML::Node& node, const std::string_view key, std::filesystem::path& out)
    {
        if (const auto n = node[std::string(key)]) out = n.as<std::string>();
    }

    void yaml_set_color(const YAML::Node& node, const std::string_view key, cv::Scalar& out)
    {
        if (const auto n = node[std::string(key)]) set_color_from_vec(n.as<std::vector<int>>(), out);
    }

    template <class E>
    void yaml_set_enum(const YAML::Node& node, const std::string_view key, E& out)
    {
        if (const auto n = node[std::string(key)])
        {
            if (auto v = magic_enum::enum_cast<E>(n.as<std::string>())) out = *v;
        }
    }

    void yaml_set_size(const YAML::Node& node, const std::string_view key, cv::Size& out)
    {
        if (const auto n = node[std::string(key)])
        {
            const auto arr = n.as<std::vector<int>>();
            if (arr.size() == 2)
            {
                out.width  = arr[0];
                out.height = arr[1];
            }
        }
    }
}

boost::program_options::options_description zenslam::options::description()
{
    const options options;

    boost::program_options::options_description description { "options" };

    description.add_options()
            (
                "options-file",
                boost::program_options::value<std::string>()->default_value(options.file),
                "options file"
            )
            (
                "log-level",
                boost::program_options::value<std::string>()->default_value(utils::log_levels_to_string[options.log_level]),
                ("log level - pick one of: " + utils::to_string(magic_enum::enum_names<spdlog::level::level_enum>())).c_str()
            )
            ("help,h", "Show help")("version,v", "Show version");

    description.add(folder::description());
    description.add(slam::description());

    return description;
}

zenslam::options zenslam::options::parse(const int argc, char** argv)
{
    options options { };

    const auto& description = options::description(); // inlining this can cause argument parse failures, don't know why!
    const auto& parsed      = parse_command_line(argc, argv, description);
    auto        map         = boost::program_options::variables_map();

    store(parsed, map);
    notify(map);

    if (parsed.options.empty() || map.contains("help"))
    {
        options.verb = verb::HELP;
    }

    if (map.contains("version"))
    {
        options.verb = verb::VERSION;
    }

    std::map<std::string, boost::program_options::basic_option<char>> options_map { };
    for (auto& option: parsed.options)
    {
        options_map[option.string_key] = option;
    }

    if (options_map.contains("options-file")) options = parse(map["options-file"].as<std::string>());

    set_path_if_provided(options_map, map, "folder-root", options.folder.root);
    set_path_if_provided(options_map, map, "folder-left", options.folder.left);
    set_path_if_provided(options_map, map, "folder-right", options.folder.right);
    set_path_if_provided(options_map, map, "folder-output", options.folder.output);
    set_if_provided(options_map, map, "folder-timescale", options.folder.timescale);
    set_path_if_provided(options_map, map, "calibration-file", options.folder.calibration_file);
    set_path_if_provided(options_map, map, "groundtruth-file", options.folder.groundtruth_file);
    set_path_if_provided(options_map, map, "imu-calibration-file", options.folder.imu_calibration_file);
    set_path_if_provided(options_map, map, "options-file", options.file);
    set_if_provided(options_map, map, "log-level", options.log_level);
    set_if_provided(options_map, map, "log-pattern", options.log_pattern);
    set_if_provided(options_map, map, "stereo-rectify", options.slam.stereo_rectify);
    set_if_provided(options_map, map, "use-parallel-detector", options.slam.use_parallel_detector);
    set_if_provided(options_map, map, "epipolar-threshold", options.slam.epipolar_threshold);
    set_if_provided(options_map, map, "fast-threshold", options.slam.fast_threshold);
    set_if_provided(options_map, map, "threshold-3d3d", options.slam.threshold_3d3d);
    set_if_provided(options_map, map, "threshold-3d2d", options.slam.threshold_3d2d);
    set_if_provided(options_map, map, "show-keypoints", options.slam.show_keypoints);
    set_if_provided(options_map, map, "show-keylines", options.slam.show_keylines);
    set_if_provided(options_map, map, "keyline-thickness", options.slam.keyline_thickness);
    set_if_provided(options_map, map, "triangulation-min-disparity", options.slam.triangulation_min_disparity);
    set_if_provided(options_map, map, "triangulation-min-angle", options.slam.triangulation_min_angle);
    set_if_provided(options_map, map, "triangulation-reprojection-threshold", options.slam.triangulation_reprojection_threshold);
    set_if_provided(options_map, map, "triangulation-min-depth", options.slam.triangulation_min_depth);
    set_if_provided(options_map, map, "triangulation-max-depth", options.slam.triangulation_max_depth);
    set_color_if_provided(options_map, map, "keyline-single-color", options.slam.keyline_single_color);
    set_color_if_provided(options_map, map, "keyline-match-color", options.slam.keyline_match_color);
    set_enum_if_provided(options_map, map, "feature", options.slam.feature);
    set_enum_if_provided(options_map, map, "descriptor", options.slam.descriptor);

    return options;
}

zenslam::options zenslam::options::parse(const std::filesystem::path& path)
{
    options options { };

    options.file = path;

    try
    {
        auto config = YAML::LoadFile(path.string());

        if (const auto& application = config["application"])
        {
            if (const auto x = application["log_level"]) options.log_level = utils::log_levels_from_string[x.as<std::string>()];
            yaml_set_if_present(application, "log_pattern", options.log_pattern);
        }

        if (const auto& folder = config["folder"])
        {
            yaml_set_path(folder, "root", options.folder.root);
            yaml_set_path(folder, "left", options.folder.left);
            yaml_set_path(folder, "right", options.folder.right);
            yaml_set_if_present(folder, "timescale", options.folder.timescale);
            yaml_set_path(folder, "calibration_file", options.folder.calibration_file);
            yaml_set_path(folder, "groundtruth_file", options.folder.groundtruth_file);
            yaml_set_path(folder, "output", options.folder.output);
            yaml_set_path(folder, "imu_calibration_file", options.folder.imu_calibration_file);
        }

        if (const auto& slam = config["slam"])
        {
            yaml_set_size(slam, "cell_size", options.slam.cell_size);
            yaml_set_if_present(slam, "clahe_enabled", options.slam.clahe_enabled);
            yaml_set_if_present(slam, "use_parallel_detector", options.slam.use_parallel_detector);
            yaml_set_if_present(slam, "stereo_rectify", options.slam.stereo_rectify);
            yaml_set_if_present(slam, "epipolar_threshold", options.slam.epipolar_threshold);
            yaml_set_enum(slam, "feature", options.slam.feature);
            yaml_set_enum(slam, "descriptor", options.slam.descriptor);
            yaml_set_if_present(slam, "fast_threshold", options.slam.fast_threshold);
            yaml_set_size(slam, "klt_window_size", options.slam.klt_window_size);
            yaml_set_if_present(slam, "klt_max_level", options.slam.klt_max_level);
            yaml_set_if_present(slam, "threshold_3d3d", options.slam.threshold_3d3d);
            yaml_set_if_present(slam, "threshold_3d2d", options.slam.threshold_3d2d);
            yaml_set_if_present(slam, "show_keypoints", options.slam.show_keypoints);
            yaml_set_if_present(slam, "show_keylines", options.slam.show_keylines);
            yaml_set_color(slam, "keyline_single_color", options.slam.keyline_single_color);
            yaml_set_color(slam, "keyline_match_color", options.slam.keyline_match_color);
            yaml_set_if_present(slam, "keyline_thickness", options.slam.keyline_thickness);
            yaml_set_if_present(slam, "triangulation_min_disparity", options.slam.triangulation_min_disparity);
            yaml_set_if_present(slam, "triangulation_min_angle", options.slam.triangulation_min_angle);
            yaml_set_if_present(slam, "triangulation_reprojection_threshold", options.slam.triangulation_reprojection_threshold);
        }
    }
    catch (const YAML::Exception& e)
    {
        SPDLOG_ERROR("Failed to load config file: {}", e.what());
    }

    return options;
}

boost::program_options::options_description zenslam::options::folder::description()
{
    const folder folder;

    boost::program_options::options_description description { "folder options" };

    description.add_options()
    (
        "folder-root",
        boost::program_options::value<std::string>()->default_value(folder.root),
        "Root folder"
    )
    (
        "folder-left",
        boost::program_options::value<std::string>()->default_value(folder.left),
        "Left folder relative to root (or absolute)"
    )
    (
        "folder-right",
        boost::program_options::value<std::string>()->default_value(folder.right),
        "Right folder relative to root (or absolute)"
    )
    (
        "folder-timescale",
        boost::program_options::value<double>()->default_value(folder.timescale),
        "Timescale for folder timestamps"
    )
    (
        "calibration-file",
        boost::program_options::value<std::string>()->default_value(folder.calibration_file),
        "calibration file path"
    )
    (
        "groundtruth-file",
        boost::program_options::value<std::string>()->default_value(folder.groundtruth_file),
        "groundtruth file path"
    )
    (
        "imu-calibration-file",
        boost::program_options::value<std::string>()->default_value(folder.imu_calibration_file),
        "IMU calibration file path"
    )
    (
        "folder-output",
        boost::program_options::value<std::string>()->default_value(folder.output),
        "Output folder for results"
    );

    return description;
}

void zenslam::options::folder::validate() const
{
    if (timescale <= 0.0) throw std::invalid_argument("folder.timescale must be > 0");

    // Non-fatal warnings for non-existent paths
    auto resolve = [this](const std::filesystem::path& p) -> std::filesystem::path
    {
        return p.is_absolute() ? p : root / p;
    };

    const auto root_path = root;
    if (!root_path.empty() && !std::filesystem::exists(root_path))
    {
        SPDLOG_WARN("folder.root does not exist: {}", root_path.string());
    }

    const auto left_path = resolve(left);
    if (!std::filesystem::exists(left_path))
    {
        SPDLOG_WARN("folder.left path does not exist: {}", left_path.string());
    }

    const auto right_path = resolve(right);
    if (!std::filesystem::exists(right_path))
    {
        SPDLOG_WARN("folder.right path does not exist: {}", right_path.string());
    }

    const auto calib_path = resolve(calibration_file);
    if (!std::filesystem::exists(calib_path))
    {
        SPDLOG_WARN("folder.calibration_file not found: {}", calib_path.string());
    }

    const auto gt_path = resolve(groundtruth_file);
    if (!std::filesystem::exists(gt_path))
    {
        SPDLOG_WARN("folder.groundtruth_file not found: {}", gt_path.string());
    }

    const auto imu_path = resolve(imu_calibration_file);
    if (!std::filesystem::exists(imu_path))
    {
        SPDLOG_WARN("folder.imu_calibration_file not found: {}", imu_path.string());
    }
}

void zenslam::options::folder::print() const
{
    SPDLOG_INFO("folder root: {}", root.string());
    SPDLOG_INFO("folder left: {}", left.string());
    SPDLOG_INFO("folder right: {}", right.string());
    SPDLOG_INFO("folder output: {}", output.string());
    SPDLOG_INFO("folder timescale: {}", timescale);
    SPDLOG_INFO("calibration file: {}", calibration_file.string());
    SPDLOG_INFO("groundtruth file: {}", groundtruth_file.string());
    SPDLOG_INFO("IMU calibration file: {}", imu_calibration_file.string());
}

boost::program_options::options_description zenslam::options::slam::description()
{
    const options opts_default;

    boost::program_options::options_description desc { "slam options" };

    desc.add_options()
    (
        "clahe-enabled",
        boost::program_options::bool_switch()->default_value(opts_default.slam.clahe_enabled),
        "CLAHE enabled"
    )
    (
        "use-parallel-detector",
        boost::program_options::bool_switch()->default_value(opts_default.slam.use_parallel_detector),
        "Use parallel grid detector (detect_keypoints_par)"
    )
    (
        "stereo-rectify",
        boost::program_options::bool_switch()->default_value(opts_default.slam.stereo_rectify),
        "Enable stereo rectification"
    )
    (
        "epipolar-threshold",
        boost::program_options::value<double>()->default_value(opts_default.slam.epipolar_threshold),
        "Epipolar threshold"
    )
    (
        "feature",
        boost::program_options::value<std::string>()->default_value(std::string(magic_enum::enum_name(opts_default.slam.feature))),
        ("feature type - pick one of: " + utils::to_string(magic_enum::enum_names<feature_type>())).c_str()
    )
    (
        "descriptor",
        boost::program_options::value<std::string>()->default_value(std::string(magic_enum::enum_name(opts_default.slam.descriptor))),
        ("descriptor type - pick one of: " + utils::to_string(magic_enum::enum_names<descriptor_type>())).c_str()
    )
    (
        "fast-threshold",
        boost::program_options::value<double>()->default_value(opts_default.slam.fast_threshold),
        "FAST threshold"
    )
    (
        "threshold-3d3d",
        boost::program_options::value<double>()->default_value(opts_default.slam.threshold_3d3d),
        "3D-3D RANSAC pose estimation threshold in meters"
    )
    (
        "threshold-3d2d",
        boost::program_options::value<double>()->default_value(opts_default.slam.threshold_3d2d),
        "3D-2D RANSAC pose estimation threshold in pixels"
    )
    (
        "show-keypoints",
        boost::program_options::bool_switch()->default_value(opts_default.slam.show_keypoints),
        "Show keypoints in visualization"
    )
    (
        "show-keylines",
        boost::program_options::bool_switch()->default_value(opts_default.slam.show_keylines),
        "Show keylines in visualization"
    )
    (
        "keyline-single-color",
        boost::program_options::value<std::vector<int>>()->multitoken()->default_value(std::vector<int> { 0, 255, 0 }, "0 255 0"),
        "Keyline single color (B G R)"
    )
    (
        "keyline-match-color",
        boost::program_options::value<std::vector<int>>()->multitoken()->default_value(std::vector<int> { 0, 0, 255 }, "0 0 255"),
        "Keyline match color (B G R)"
    )
    (
        "keyline-thickness",
        boost::program_options::value<int>()->default_value(opts_default.slam.keyline_thickness),
        "Keyline line thickness (pixels)"
    )
    (
        "keyline-min-disparity-px",
        boost::program_options::value<double>()->default_value(opts_default.slam.triangulation_min_disparity),
        "Keyline min average disparity across endpoints (pixels)"
    )
    (
        "keyline-min-triangulation-angle-deg",
        boost::program_options::value<double>()->default_value(opts_default.slam.triangulation_min_angle),
        "Keyline min triangulation angle at endpoints (degrees)"
    )
    (
        "triangulation_reprojection_threshold",
        boost::program_options::value<double>()->default_value(opts_default.slam.triangulation_reprojection_threshold),
        "Keyline max average reprojection error across endpoints (pixels)"
    );

    return desc;
}

void zenslam::options::slam::validate() const
{
    if (keyline_thickness < 1) throw std::invalid_argument("slam.keyline_thickness must be >= 1");
    if (epipolar_threshold < 0.0) throw std::invalid_argument("slam.epipolar_threshold must be >= 0");
    if (triangulation_min_disparity < 0.0) throw std::invalid_argument("slam.keyline_min_disparity_px must be >= 0");
    if (triangulation_min_angle < 0.0) throw std::invalid_argument("slam.keyline_min_triangulation_angle_deg must be >= 0");
    if (triangulation_reprojection_threshold <= 0.0) throw std::invalid_argument("slam.triangulation_reprojection_threshold must be > 0");
    if (triangulation_min_depth <= 0.0 || triangulation_max_depth <= 0.0 || triangulation_min_depth >= triangulation_max_depth)
        throw std::invalid_argument
                ("slam depth range invalid: ensure 0 < min_depth < max_depth");
    if (klt_window_size.width <= 0 || klt_window_size.height <= 0) throw std::invalid_argument("slam.klt_window_size must be positive");
    if (klt_max_level < 0) throw std::invalid_argument("slam.klt_max_level must be >= 0");
}

void zenslam::options::slam::print() const
{
    SPDLOG_INFO("cell size: [{}, {}]", cell_size.width, cell_size.height);
    SPDLOG_INFO("CLAHE enabled: {}", clahe_enabled ? "true" : "false");
    SPDLOG_INFO("use parallel detector: {}", use_parallel_detector ? "true" : "false");
    SPDLOG_INFO("stereo rectify: {}", stereo_rectify ? "true" : "false");
    SPDLOG_INFO("epipolar threshold: {}", epipolar_threshold);
    SPDLOG_INFO("feature type: {}", magic_enum::enum_name(feature));
    SPDLOG_INFO("descriptor type: {}", magic_enum::enum_name(descriptor));
    SPDLOG_INFO("fast threshold: {}", fast_threshold);
    SPDLOG_INFO("klt window size: [{}, {}]", klt_window_size.width, klt_window_size.height);
    SPDLOG_INFO("klt max level: {}", klt_max_level);
    SPDLOG_INFO("3D-3D RANSAC threshold: {} m", threshold_3d3d);
    SPDLOG_INFO("3D-2D RANSAC threshold: {} px", threshold_3d2d);
    SPDLOG_INFO("show keypoints: {}", show_keypoints ? "true" : "false");
    SPDLOG_INFO("show keylines: {}", show_keylines ? "true" : "false");
    SPDLOG_INFO("keyline single color (BGR): [{}, {}, {}]", keyline_single_color[0], keyline_single_color[1], keyline_single_color[2]);
    SPDLOG_INFO("keyline match color  (BGR): [{}, {}, {}]", keyline_match_color[0], keyline_match_color[1], keyline_match_color[2]);
    SPDLOG_INFO("keyline thickness: {}", keyline_thickness);
    SPDLOG_INFO("keyline min disparity (px): {}", triangulation_min_disparity);
    SPDLOG_INFO("keyline min triangulation angle (deg): {}", triangulation_min_angle);
    SPDLOG_INFO("keyline reprojection threshold (px): {}", triangulation_reprojection_threshold);
}

void zenslam::options::validate() const
{
    folder.validate();
    slam.validate();
}

void zenslam::options::print() const
{
    SPDLOG_INFO("file: {}", file.string());
    SPDLOG_INFO("log level: {}", magic_enum::enum_name(log_level));
    SPDLOG_INFO("log pattern: {}", log_pattern);

    folder.print();
    slam.print();
}
