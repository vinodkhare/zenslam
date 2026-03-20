#include "imgui_controls_window.h"

#include <imgui.h>
#include <implot.h>

#include <opencv2/core/types.hpp>

#include <zenslam/utils/utils.h>

namespace zenslam
{
    imgui_controls_window::imgui_controls_window(gui_options& gui_options) : _gui_options(gui_options) {}

    void imgui_controls_window::load_fonts()
    {
        ImFontConfig cfg;
        cfg.OversampleH = 2;
        cfg.OversampleV = 2;
        // Index 0 = Menlo Regular within the .ttc collection
        cfg.FontNo  = 0;
        s_font_mono = ImGui::GetIO().Fonts->AddFontFromFileTTF("/System/Library/Fonts/Menlo.ttc", 13.0f, &cfg);
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
        _indexing_history.push_back(std::chrono::duration<double>(system.durations.indexing).count());
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

        if (s_font_mono)
            ImGui::PushFont(s_font_mono);

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
            const auto  euler_rad   = zenslam::utils::matrix_to_euler(pose.rotation());
            const auto  euler_deg   = euler_rad * (180.0 / CV_PI);

            ImGui::TextUnformatted("Pose");
            constexpr ImGuiTableFlags table_flags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg;
            if (ImGui::BeginTable("pose_table", 4, table_flags))
            {
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Pos (m)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%+9.3f", translation[0]);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%+9.3f", translation[1]);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%+9.3f", translation[2]);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Rot (deg)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%+8.2f", euler_deg[0]); // roll
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%+8.2f", euler_deg[1]); // pitch
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%+8.2f", euler_deg[2]); // yaw

                ImGui::EndTable();
            }
            ImGui::Spacing();
        }

        // IMU Data
        if (!system[1].imu_data.empty())
        {
            const auto& imu = system[1].imu_data.back();

            ImGui::Text("IMU  (%zu samples)", system[1].imu_data.size());
            constexpr ImGuiTableFlags table_flags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg;
            if (ImGui::BeginTable("imu_table", 4, table_flags))
            {
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 90.0f);
                ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Gyro (rad/s)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%+9.4f", imu.gyr[0]);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%+9.4f", imu.gyr[1]);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%+9.4f", imu.gyr[2]);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("Accel (m/s\u00b2)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%+9.4f", imu.acc[0]);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%+9.4f", imu.acc[1]);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%+9.4f", imu.acc[2]);

                ImGui::EndTable();
            }
            ImGui::Spacing();
        }
        else
        {
            ImGui::TextDisabled("IMU: no data");
            ImGui::Spacing();
        }

        // Map points count
        ImGui::Text("Map Points:  %zu", system.points3d.size());

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
        ImVec4      color_rgba(static_cast<float>(s[2]) / 255.0f, static_cast<float>(s[1]) / 255.0f, static_cast<float>(s[0]) / 255.0f, 1.0f);

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
            ImPlot::PlotLine("Processing", _time_history.data(), _processing_history.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Tracking", _time_history.data(), _tracking_history.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Estimation", _time_history.data(), _estimation_history.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Indexing", _time_history.data(), _indexing_history.data(), static_cast<int>(_time_history.size()));
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
            ImPlot::PlotLine("KP L", _time_history.data(), _point_history.features_l.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("KP R", _time_history.data(), _point_history.features_r.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Tracked L", _time_history.data(), _point_history.features_l_tracked.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Tracked R", _time_history.data(), _point_history.features_r_tracked.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("New L", _time_history.data(), _point_history.features_l_new.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("New R", _time_history.data(), _point_history.features_r_new.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("KP Total", _time_history.data(), _point_history.features_total.data(), static_cast<int>(_time_history.size()));

            // Matches & 3D
            ImPlot::PlotLine("Matches", _time_history.data(), _point_history.matches_stereo.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Triangulated", _time_history.data(), _point_history.triangulated_3d.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Map Points", _time_history.data(), _point_history.map_total.data(), static_cast<int>(_time_history.size()));

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
            ImPlot::PlotLine("KL L", _time_history.data(), _line_history.features_l.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("KL R", _time_history.data(), _line_history.features_r.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Tracked L", _time_history.data(), _line_history.features_l_tracked.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Tracked R", _time_history.data(), _line_history.features_r_tracked.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("New L", _time_history.data(), _line_history.features_l_new.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("New R", _time_history.data(), _line_history.features_r_new.data(), static_cast<int>(_time_history.size()));

            // Matches & 3D
            ImPlot::PlotLine("KL Matches", _time_history.data(), _line_history.matches_stereo.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Lines 3D", _time_history.data(), _line_history.triangulated_3d.data(), static_cast<int>(_time_history.size()));
            ImPlot::PlotLine("Map Lines", _time_history.data(), _line_history.map_total.data(), static_cast<int>(_time_history.size()));

            ImPlot::EndPlot();
        }

        if (s_font_mono)
            ImGui::PopFont();
    }
} // namespace zenslam
