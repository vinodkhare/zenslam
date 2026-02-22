#include "imgui_controls_window.h"

#include <imgui.h>
#include <implot.h>

#include <opencv2/core/types.hpp>

#include <zenslam/utils/utils.h>

namespace zenslam
{
    imgui_controls_window::imgui_controls_window(gui_options& gui_options) :
        _gui_options(gui_options)
    {
    }

    void imgui_controls_window::initialize()
    {
        if (_initialized)
            return;

        ImPlot::CreateContext();
        _initialized = true;
    }

    void imgui_controls_window::update_history(const frame::system& system)
    {
        // Update timing history for plots
        _time_history.push_back(system[1].timestamp);
        _wait_history.push_back(std::chrono::duration<double>(system.durations.wait).count());
        _processing_history.push_back(std::chrono::duration<double>(system.durations.processing).count());
        _tracking_history.push_back(std::chrono::duration<double>(system.durations.tracking).count());
        _estimation_history.push_back(std::chrono::duration<double>(system.durations.estimation).count());
        _total_history.push_back(std::chrono::duration<double>(system.durations.total).count());

        // Update point counts history
        _point_history.features_l.push_back(static_cast<double>(system.counts.points.features_l));
        _point_history.features_r.push_back(static_cast<double>(system.counts.points.features_r));
        _point_history.features_l_tracked.push_back(static_cast<double>(system.counts.points.features_l_tracked));
        _point_history.features_r_tracked.push_back(static_cast<double>(system.counts.points.features_r_tracked));
        _point_history.features_l_new.push_back(static_cast<double>(system.counts.points.features_l_new));
        _point_history.features_r_new.push_back(static_cast<double>(system.counts.points.features_r_new));
        _point_history.features_total.push_back(static_cast<double>(system.counts.points.features_total));
        _point_history.matches_stereo.push_back(static_cast<double>(system.counts.points.matches_stereo));
        _point_history.triangulated_3d.push_back(static_cast<double>(system.counts.points.triangulated_3d));
        _point_history.map_total.push_back(static_cast<double>(system.counts.map_points));

        // Update line counts history
        _line_history.features_l.push_back(static_cast<double>(system.counts.lines.features_l));
        _line_history.features_r.push_back(static_cast<double>(system.counts.lines.features_r));
        _line_history.features_l_tracked.push_back(static_cast<double>(system.counts.lines.features_l_tracked));
        _line_history.features_r_tracked.push_back(static_cast<double>(system.counts.lines.features_r_tracked));
        _line_history.features_l_new.push_back(static_cast<double>(system.counts.lines.features_l_new));
        _line_history.features_r_new.push_back(static_cast<double>(system.counts.lines.features_r_new));
        _line_history.features_total.push_back(static_cast<double>(system.counts.lines.features_total));
        _line_history.matches_stereo.push_back(static_cast<double>(system.counts.lines.matches_stereo));
        _line_history.triangulated_3d.push_back(static_cast<double>(system.counts.lines.triangulated_3d));
        _line_history.map_total.push_back(static_cast<double>(system.counts.map_lines));
    }

    void imgui_controls_window::render(const frame::system& system)
    {
        // ReSharper disable once CppDFAConstantConditions
        if (!_visible)
            // ReSharper disable once CppDFAUnreachableCode
            return;

        if (!_initialized)
            initialize();

        ImGui::Text("Hello Metal!");
        ImGui::Separator();

        // Current Frame Information
        ImGui::Text("Current Frame Info");
        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 6.0f));
        ImGui::BeginChild("frame_info_section", ImVec2(0, 200.0f), true);

        // Pose Information
        if (!std::isnan(system[1].timestamp))
        {
            const auto& pose        = system[1].pose;
            const auto& translation = pose.translation();

            // Extract Euler angles from rotation matrix (returns [roll, pitch, yaw] in radians)
            const auto euler_rad = zenslam::utils::matrix_to_euler(pose.rotation());

            // Convert to degrees
            const auto euler_deg = euler_rad * (180.0 / CV_PI);

            ImGui::Text("Pose:");
            ImGui::Text("  Position [x, y, z]: [%.3f, %.3f, %.3f] m", translation[0], translation[1], translation[2]);
            ImGui::Text("  Euler [roll, pitch, yaw]: [%.2f, %.2f, %.2f] deg", euler_deg[0], euler_deg[1], euler_deg[2]);
            ImGui::Spacing();
        }

        // IMU Data
        if (!system[1].imu_data.empty())
        {
            const auto& imu = system[1].imu_data.back(); // Show latest IMU measurement

            ImGui::Text("IMU Data (%zu samples):", system[1].imu_data.size());
            ImGui::Text("  Angular Vel [wx, wy, wz]: [%.3f, %.3f, %.3f] rad/s", imu.gyr[0], imu.gyr[1], imu.gyr[2]);
            ImGui::Text("  Linear Acc  [ax, ay, az]: [%.3f, %.3f, %.3f] m/s\u00b2", imu.acc[0], imu.acc[1], imu.acc[2]);
        }
        else
        {
            ImGui::Text("IMU Data: No data available");
        }

        ImGui::EndChild();
        ImGui::PopStyleVar(2);

        ImGui::Separator();
        ImGui::Text("Visualization Options");
        ImGui::Spacing();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 6.0f));
        ImGui::BeginChild("viz_section", ImVec2(0, 160.0f), true);

        ImGui::Checkbox("Show Keypoints", &_gui_options.show_keypoints);
        ImGui::Checkbox("Show Keylines", &_gui_options.show_keylines);

        auto point_cloud_opacity = static_cast<float>(_gui_options.point_cloud_opacity);
        if (ImGui::SliderFloat("Point Cloud Opacity", &point_cloud_opacity, 0.0f, 1.0f, "%.2f"))
        {
            _gui_options.point_cloud_opacity = point_cloud_opacity;
        }

        ImGui::SliderFloat("Point Size", &_gui_options.point_size, 1.0f, 20.0f, "%.1f");

        // Color picker for keylines (single keyline color)
        const auto& s = _gui_options.keyline_single_color; // B, G, R
        ImVec4      color_rgba
        (
            static_cast<float>(s[2]) / 255.0f,
            // R
            static_cast<float>(s[1]) / 255.0f,
            // G
            static_cast<float>(s[0]) / 255.0f,
            // B
            1.0f
        );

        if (ImGui::ColorEdit3("Keyline Color", reinterpret_cast<float*>(&color_rgba)))
        {
            const auto r                      = static_cast<int>(std::round(color_rgba.x * 255.0f));
            const auto g                      = static_cast<int>(std::round(color_rgba.y * 255.0f));
            const auto b                      = static_cast<int>(std::round(color_rgba.z * 255.0f));
            _gui_options.keyline_single_color = cv::Scalar(b, g, r);
        }

        ImGui::EndChild();
        ImGui::PopStyleVar(2);

        // Frame Duration Plots
        ImGui::Separator();
        ImGui::Text("Frame Durations");
        ImGui::Spacing();

        constexpr auto interval = 10; // seconds

        if (!_time_history.empty() && ImPlot::BeginPlot("Processing Times", ImVec2(-1, 240)))
        {
            // Lock Y axis to disable zooming/panning on Y
            ImPlot::SetupAxes("Time (s)", "Duration (s)", ImPlotAxisFlags_None, ImPlotAxisFlags_Lock);

            ImPlot::SetupAxisLimits(ImAxis_X1, _time_history.back() - interval, _time_history.back(), ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.1, ImGuiCond_Once);
            ImPlot::SetupLegend(ImPlotLocation_NorthEast);
            ImPlot::SetupAxisFormat(ImAxis_X1, "%.2f");
            ImPlot::SetupAxisFormat(ImAxis_Y1, "%.4f");

            ImPlot::PlotLine("Wait", _time_history.data(), _wait_history.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine
            (
                "Processing",
                _time_history.data(),
                _processing_history.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Tracking",
                _time_history.data(),
                _tracking_history.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Estimation",
                _time_history.data(),
                _estimation_history.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine("Total", _time_history.data(), _total_history.data(), static_cast<int>(_time_history.size()));

            ImPlot::EndPlot();
        }

        // Frame Counts Plot
        if (!_time_history.empty() && ImPlot::BeginPlot("Frame Counts", ImVec2(-1, 320)))
        {
            ImPlot::SetupAxes("Time (s)", "Count", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
            ImPlot::SetupAxisLimits(ImAxis_X1, _time_history.back() - interval, _time_history.back(), ImGuiCond_Always);
            ImPlot::SetupLegend(ImPlotLocation_NorthEast);
            ImPlot::SetupAxisFormat(ImAxis_X1, "%.2f");

            // Keypoints
            ImPlot::PlotLine
            (
                "KP L",
                _time_history.data(),
                _point_history.features_l.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "KP R",
                _time_history.data(),
                _point_history.features_r.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Tracked L",
                _time_history.data(),
                _point_history.features_l_tracked.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Tracked R",
                _time_history.data(),
                _point_history.features_r_tracked.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "New L",
                _time_history.data(),
                _point_history.features_l_new.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "New R",
                _time_history.data(),
                _point_history.features_r_new.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "KP Total",
                _time_history.data(),
                _point_history.features_total.data(),
                static_cast<int>(_time_history.size())
            );

            // Matches & 3D
            ImPlot::PlotLine
            (
                "Matches",
                _time_history.data(),
                _point_history.matches_stereo.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Triangulated",
                _time_history.data(),
                _point_history.triangulated_3d.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Map Points",
                _time_history.data(),
                _point_history.map_total.data(),
                static_cast<int>(_time_history.size())
            );

            ImPlot::EndPlot();
        }

        // Keyline Counts Plot
        if (!_time_history.empty() && ImPlot::BeginPlot("Keyline Counts", ImVec2(-1, 320)))
        {
            ImPlot::SetupAxes("Time (s)", "Count", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
            ImPlot::SetupAxisLimits(ImAxis_X1, _time_history.back() - interval, _time_history.back(), ImGuiCond_Always);
            ImPlot::SetupLegend(ImPlotLocation_NorthEast);
            ImPlot::SetupAxisFormat(ImAxis_X1, "%.2f");

            // Keylines
            ImPlot::PlotLine
            (
                "KL L",
                _time_history.data(),
                _line_history.features_l.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "KL R",
                _time_history.data(),
                _line_history.features_r.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Tracked L",
                _time_history.data(),
                _line_history.features_l_tracked.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Tracked R",
                _time_history.data(),
                _line_history.features_r_tracked.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "New L",
                _time_history.data(),
                _line_history.features_l_new.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "New R",
                _time_history.data(),
                _line_history.features_r_new.data(),
                static_cast<int>(_time_history.size())
            );

            // Matches & 3D
            ImPlot::PlotLine
            (
                "KL Matches",
                _time_history.data(),
                _line_history.matches_stereo.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Lines 3D",
                _time_history.data(),
                _line_history.triangulated_3d.data(),
                static_cast<int>(_time_history.size())
            );
            ImPlot::PlotLine
            (
                "Map Lines",
                _time_history.data(),
                _line_history.map_total.data(),
                static_cast<int>(_time_history.size())
            );

            ImPlot::EndPlot();
        }
    }
} // namespace zenslam
