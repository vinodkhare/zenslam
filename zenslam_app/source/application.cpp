#include "application.h"

#include <imgui.h>
#include <implot.h>

#include <opencv2/highgui.hpp>

#include <zenslam/utils.h>

#include <utility>

#include "zenslam/utils_opencv.h"

// VTK includes (kept in .cpp)
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkCellArray.h>
#include <vtkImageImport.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLine.h>
#include <vtkMatrix4x4.h>
#include <vtkPlaneSource.h>
#include <vtkPolyLine.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>

zenslam::application::application(options options) : _options { std::move(options) }
{
    _slam_thread.on_frame += [this](const frame::system& frame)
    {
        std::lock_guard lock { _mutex };
        _system = frame;

        // Update timing history for plots
        {
            _time_history.push_back(_system[1].timestamp);
            _wait_history.push_back(std::chrono::duration<double>(_system.durations.wait).count());
            _processing_history.push_back(std::chrono::duration<double>(_system.durations.processing).count());
            _tracking_history.push_back(std::chrono::duration<double>(_system.durations.tracking).count());
            _estimation_history.push_back(std::chrono::duration<double>(_system.durations.estimation).count());
            _total_history.push_back(std::chrono::duration<double>(_system.durations.total).count());

            // Update point counts history
            _point_history.features_l.push_back(static_cast<double>(_system.counts.points.features_l));
            _point_history.features_r.push_back(static_cast<double>(_system.counts.points.features_r));
            _point_history.features_l_tracked.push_back(static_cast<double>(_system.counts.points.features_l_tracked));
            _point_history.features_r_tracked.push_back(static_cast<double>(_system.counts.points.features_r_tracked));
            _point_history.features_l_new.push_back(static_cast<double>(_system.counts.points.features_l_new));
            _point_history.features_r_new.push_back(static_cast<double>(_system.counts.points.features_r_new));
            _point_history.features_total.push_back(static_cast<double>(_system.counts.points.features_total));
            _point_history.matches_stereo.push_back(static_cast<double>(_system.counts.points.matches_stereo));
            _point_history.triangulated_3d.push_back(static_cast<double>(_system.counts.points.triangulated_3d));
            _point_history.map_total.push_back(static_cast<double>(_system.counts.map_points));

            // Update line counts history
            _line_history.features_l.push_back(static_cast<double>(_system.counts.lines.features_l));
            _line_history.features_r.push_back(static_cast<double>(_system.counts.lines.features_r));
            _line_history.features_l_tracked.push_back(static_cast<double>(_system.counts.lines.features_l_tracked));
            _line_history.features_r_tracked.push_back(static_cast<double>(_system.counts.lines.features_r_tracked));
            _line_history.features_l_new.push_back(static_cast<double>(_system.counts.lines.features_l_new));
            _line_history.features_r_new.push_back(static_cast<double>(_system.counts.lines.features_r_new));
            _line_history.features_total.push_back(static_cast<double>(_system.counts.lines.features_total));
            _line_history.matches_stereo.push_back(static_cast<double>(_system.counts.lines.matches_stereo));
            _line_history.triangulated_3d.push_back(static_cast<double>(_system.counts.lines.triangulated_3d));
            _line_history.map_total.push_back(static_cast<double>(_system.counts.map_lines));
        }

        // Update trajectory history
        {
            const auto& pos = _system[1].pose.translation();
            _trajectory_estimated.emplace_back(pos[0], pos[1], pos[2]);

            const auto& pos_gt = _system[1].pose_gt.translation();
            _trajectory_gt.emplace_back(pos_gt[0], pos_gt[1], pos_gt[2]);
        }
    };

    _reader_thread.on_frame += [this](const frame::sensor& frame) { _slam_thread.enqueue(frame); };
}

void zenslam::application::render()
{
    frame::system system { };
    {
        std::lock_guard lock { _mutex };
        system = _system;
    }

    if (!is_renderable(system))
        return;

    // 2D views
    draw_spatial_matches(system);
    draw_temporal_matches(system);

    // 3D scene (native VTK window)
    draw_scene_vtk(system);

    // UI controls
    draw_viz_controls();

    cv::waitKey(1);
}

bool zenslam::application::is_renderable(const frame::system& system)
{
    // Require undistorted images and some keypoints to render informative views
    return !system[0].undistorted[0].empty() && !system[1].undistorted[0].empty() && !system[1].keypoints[0].empty();
}

void zenslam::application::draw_spatial_matches(const frame::system& system)
{
    const auto& matches_image = utils::draw_matches_spatial(system[1], system.points3d);
    cv::imshow("matches_spatial", matches_image);
    cv::setWindowTitle("matches_spatial", "matches spatial");
    cv::resizeWindow("matches_spatial", 1024, 512);
}

void zenslam::application::draw_temporal_matches(const frame::system& system) const
{
    const auto& image = utils::draw_matches_temporal(system[0], system[1], _options.slam);
    cv::namedWindow("matches_temporal");
    cv::imshow("matches_temporal", image);
    cv::setWindowTitle("matches_temporal", "matches temporal");
    cv::resizeWindow("matches_temporal", 1024, 512);
}

namespace
{
    vtkSmartPointer<vtkMatrix4x4> toVtkMatrix(const cv::Affine3d& pose)
    {
        vtkNew<vtkMatrix4x4> m;
        const auto&          P = pose.matrix; // 4x4 double
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                m->SetElement(r, c, P(r, c));
        return m;
    }
} // namespace

// Local VTK scene container
struct zenslam::application::SceneVTK
{
    vtkSmartPointer<vtkRenderer>               renderer;
    vtkSmartPointer<vtkRenderWindow>           window;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;

    // Point cloud
    vtkSmartPointer<vtkPoints>            points;
    vtkSmartPointer<vtkPolyData>          pointsPoly;
    vtkSmartPointer<vtkVertexGlyphFilter> pointsGlyph;
    vtkSmartPointer<vtkPolyDataMapper>    pointsMapper;
    vtkSmartPointer<vtkActor>             pointsActor;

    // Lines
    vtkSmartPointer<vtkPoints>         linePoints;
    vtkSmartPointer<vtkCellArray>      lineCells;
    vtkSmartPointer<vtkPolyData>       linesPoly;
    vtkSmartPointer<vtkPolyDataMapper> linesMapper;
    vtkSmartPointer<vtkActor>          linesActor;

    // Axes for camera poses
    vtkSmartPointer<vtkAxesActor> axesCam;
    vtkSmartPointer<vtkAxesActor> axesCamGt;

    // Camera frustum for current pose
    vtkSmartPointer<vtkPlaneSource>    frustumPlane;
    vtkSmartPointer<vtkPolyDataMapper> frustumMapper;
    vtkSmartPointer<vtkActor>          frustumActor;
    vtkSmartPointer<vtkTexture>        frustumTexture;
    vtkSmartPointer<vtkImageImport>    frustumImage;

    // Trajectory paths
    vtkSmartPointer<vtkPoints>         trajectoryPoints;
    vtkSmartPointer<vtkCellArray>      trajectoryCells;
    vtkSmartPointer<vtkPolyData>       trajectoryPoly;
    vtkSmartPointer<vtkPolyDataMapper> trajectoryMapper;
    vtkSmartPointer<vtkActor>          trajectoryActor;

    vtkSmartPointer<vtkPoints>         trajectoryGtPoints;
    vtkSmartPointer<vtkCellArray>      trajectoryGtCells;
    vtkSmartPointer<vtkPolyData>       trajectoryGtPoly;
    vtkSmartPointer<vtkPolyDataMapper> trajectoryGtMapper;
    vtkSmartPointer<vtkActor>          trajectoryGtActor;
};

// Ensure unique_ptr<SceneVTK> destruction sees a complete type
// Define deleter now that SceneVTK is complete
void zenslam::application::SceneVTKDeleter::operator()(const SceneVTK* p) const
{
    delete p;
}

zenslam::application::~application() = default;

void zenslam::application::draw_scene_vtk(const frame::system& system)
{
    // Init once
    if (!_vtk)
    {
        _vtk.reset(new SceneVTK());
        auto& S = *_vtk;

        S.renderer   = vtkSmartPointer<vtkRenderer>::New();
        S.window     = vtkSmartPointer<vtkRenderWindow>::New();
        S.interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

        S.window->AddRenderer(S.renderer);
        S.window->SetSize(1024, 1024); // TODO: configurable
        S.interactor->SetRenderWindow(S.window);

        // Trackball camera interaction
        const vtkNew<vtkInteractorStyleTrackballCamera> style;
        S.interactor->SetInteractorStyle(style);
        S.interactor->Initialize();

        // Background and basic axes
        S.renderer->SetBackground(0.1, 0.1, 0.12);

        S.axesCam   = vtkSmartPointer<vtkAxesActor>::New();
        S.axesCamGt = vtkSmartPointer<vtkAxesActor>::New();
        S.axesCam->SetTotalLength(0.2, 0.2, 0.2);
        S.axesCamGt->SetTotalLength(0.2, 0.2, 0.2);
        // Turn off XYZ axis labels
        S.axesCam->SetAxisLabels(0);
        S.axesCamGt->SetAxisLabels(0);
        S.renderer->AddActor(S.axesCam);
        S.renderer->AddActor(S.axesCamGt);

        // Camera frustum with image texture
        S.frustumPlane = vtkSmartPointer<vtkPlaneSource>::New();
        S.frustumPlane->SetOrigin(-0.1, -0.1, 0.2); // Image plane at z=0.2 in camera frame
        S.frustumPlane->SetPoint1(0.1, -0.1, 0.2);  // Width ~0.2
        S.frustumPlane->SetPoint2(-0.1, 0.1, 0.2);  // Height ~0.2
        S.frustumPlane->Update();

        S.frustumMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.frustumMapper->SetInputConnection(S.frustumPlane->GetOutputPort());

        S.frustumActor = vtkSmartPointer<vtkActor>::New();
        S.frustumActor->SetMapper(S.frustumMapper);

        S.frustumImage   = vtkSmartPointer<vtkImageImport>::New();
        S.frustumTexture = vtkSmartPointer<vtkTexture>::New();
        S.frustumTexture->SetInputConnection(S.frustumImage->GetOutputPort());
        S.frustumActor->SetTexture(S.frustumTexture);

        S.renderer->AddActor(S.frustumActor);

        // Points pipeline
        S.points       = vtkSmartPointer<vtkPoints>::New();
        S.pointsPoly   = vtkSmartPointer<vtkPolyData>::New();
        S.pointsGlyph  = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        S.pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.pointsActor  = vtkSmartPointer<vtkActor>::New();

        S.pointsPoly->SetPoints(S.points);
        S.pointsGlyph->SetInputData(S.pointsPoly);
        S.pointsGlyph->Update();
        S.pointsMapper->SetInputConnection(S.pointsGlyph->GetOutputPort());
        S.pointsActor->SetMapper(S.pointsMapper);
        S.pointsActor->GetProperty()->SetPointSize(4.0);
        S.renderer->AddActor(S.pointsActor);

        // Lines pipeline
        S.linePoints  = vtkSmartPointer<vtkPoints>::New();
        S.lineCells   = vtkSmartPointer<vtkCellArray>::New();
        S.linesPoly   = vtkSmartPointer<vtkPolyData>::New();
        S.linesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.linesActor  = vtkSmartPointer<vtkActor>::New();

        S.linesPoly->SetPoints(S.linePoints);
        S.linesPoly->SetLines(S.lineCells);
        S.linesMapper->SetInputData(S.linesPoly);
        S.linesActor->SetMapper(S.linesMapper);
        S.linesActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // green
        S.linesActor->GetProperty()->SetLineWidth(1.0);
        S.renderer->AddActor(S.linesActor);

        // Trajectory pipeline (estimated)
        S.trajectoryPoints  = vtkSmartPointer<vtkPoints>::New();
        S.trajectoryCells   = vtkSmartPointer<vtkCellArray>::New();
        S.trajectoryPoly    = vtkSmartPointer<vtkPolyData>::New();
        S.trajectoryMapper  = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.trajectoryActor   = vtkSmartPointer<vtkActor>::New();

        S.trajectoryPoly->SetPoints(S.trajectoryPoints);
        S.trajectoryPoly->SetLines(S.trajectoryCells);
        S.trajectoryMapper->SetInputData(S.trajectoryPoly);
        S.trajectoryActor->SetMapper(S.trajectoryMapper);
        S.trajectoryActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // red for estimated
        S.trajectoryActor->GetProperty()->SetLineWidth(3.0);
        S.renderer->AddActor(S.trajectoryActor);

        // Trajectory pipeline (ground truth)
        S.trajectoryGtPoints  = vtkSmartPointer<vtkPoints>::New();
        S.trajectoryGtCells   = vtkSmartPointer<vtkCellArray>::New();
        S.trajectoryGtPoly    = vtkSmartPointer<vtkPolyData>::New();
        S.trajectoryGtMapper  = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.trajectoryGtActor   = vtkSmartPointer<vtkActor>::New();

        S.trajectoryGtPoly->SetPoints(S.trajectoryGtPoints);
        S.trajectoryGtPoly->SetLines(S.trajectoryGtCells);
        S.trajectoryGtMapper->SetInputData(S.trajectoryGtPoly);
        S.trajectoryGtActor->SetMapper(S.trajectoryGtMapper);
        S.trajectoryGtActor->GetProperty()->SetColor(0.0, 1.0, 1.0); // cyan for ground truth
        S.trajectoryGtActor->GetProperty()->SetLineWidth(2.0);
        S.renderer->AddActor(S.trajectoryGtActor);
    }

    const auto& S = *_vtk;

    // Update camera axes transforms
    {
        const vtkNew<vtkTransform> tCam;
        tCam->SetMatrix(toVtkMatrix(system[1].pose));
        S.axesCam->SetUserTransform(tCam);
        S.frustumActor->SetUserTransform(tCam);

        const vtkNew<vtkTransform> tCamGt;
        tCamGt->SetMatrix(toVtkMatrix(system[1].pose_gt));
        S.axesCamGt->SetUserTransform(tCamGt);
    }

    // Update frustum texture with current undistorted image
    {
        const auto& img = system[1].undistorted[0];
        if (!img.empty() && img.type() == CV_8UC1)
        {
            // Convert grayscale to RGB for VTK
            cv::Mat imgRGB;
            cv::cvtColor(img, imgRGB, cv::COLOR_GRAY2RGB);

            S.frustumImage->SetDataSpacing(1, 1, 1);
            S.frustumImage->SetDataOrigin(0, 0, 0);
            S.frustumImage->SetWholeExtent(0, imgRGB.cols - 1, 0, imgRGB.rows - 1, 0, 0);
            S.frustumImage->SetDataExtentToWholeExtent();
            S.frustumImage->SetDataScalarTypeToUnsignedChar();
            S.frustumImage->SetNumberOfScalarComponents(3);
            S.frustumImage->SetImportVoidPointer(imgRGB.data, 1); // 1 = do not copy, use existing buffer
            S.frustumImage->Update();
            S.frustumTexture->Modified();
        }
    }

    // Update point cloud
    {
        S.points->Reset();
        const auto& pts = system.points3d.values_cast<cv::Point3d>();
        for (const auto& p : pts)
            S.points->InsertNextPoint(p.x, p.y, p.z);
        S.points->Modified();
        S.pointsPoly->Modified();
        S.pointsGlyph->Update();
        S.pointsActor->GetProperty()->SetOpacity(_point_cloud_opacity);
    }

    // Update or hide lines
    if (_options.slam.show_keylines)
    {
        S.linesActor->SetVisibility(1);
        S.linePoints->Reset();
        S.lineCells->Reset();

        for (const auto& line : system.lines3d | std::views::values)
        {
            const vtkIdType id0 = S.linePoints->InsertNextPoint(line[0].x, line[0].y, line[0].z);
            const vtkIdType id1 = S.linePoints->InsertNextPoint(line[1].x, line[1].y, line[1].z);
            vtkNew<vtkLine> l;
            l->GetPointIds()->SetId(0, id0);
            l->GetPointIds()->SetId(1, id1);
            S.lineCells->InsertNextCell(l);
        }
        S.linePoints->Modified();
        S.lineCells->Modified();
        S.linesPoly->Modified();
    }
    else
    {
        S.linesActor->SetVisibility(0);
    }

    // Update estimated trajectory
    {
        S.trajectoryPoints->Reset();
        S.trajectoryCells->Reset();

        if (_trajectory_estimated.size() >= 2)
        {
            // Add all points
            for (const auto& pt : _trajectory_estimated)
            {
                S.trajectoryPoints->InsertNextPoint(pt.x, pt.y, pt.z);
            }

            // Create polyline connecting all points
            vtkNew<vtkPolyLine> polyLine;
            polyLine->GetPointIds()->SetNumberOfIds(_trajectory_estimated.size());
            for (size_t i = 0; i < _trajectory_estimated.size(); ++i)
            {
                polyLine->GetPointIds()->SetId(i, i);
            }
            S.trajectoryCells->InsertNextCell(polyLine);
        }

        S.trajectoryPoints->Modified();
        S.trajectoryCells->Modified();
        S.trajectoryPoly->Modified();
    }

    // Update ground truth trajectory
    {
        S.trajectoryGtPoints->Reset();
        S.trajectoryGtCells->Reset();

        if (_trajectory_gt.size() >= 2)
        {
            // Add all points
            for (const auto& pt : _trajectory_gt)
            {
                S.trajectoryGtPoints->InsertNextPoint(pt.x, pt.y, pt.z);
            }

            // Create polyline connecting all points
            vtkNew<vtkPolyLine> polyLine;
            polyLine->GetPointIds()->SetNumberOfIds(_trajectory_gt.size());
            for (size_t i = 0; i < _trajectory_gt.size(); ++i)
            {
                polyLine->GetPointIds()->SetId(i, i);
            }
            S.trajectoryGtCells->InsertNextCell(polyLine);
        }

        S.trajectoryGtPoints->Modified();
        S.trajectoryGtCells->Modified();
        S.trajectoryGtPoly->Modified();
    }

    // Render one frame; process user events without blocking
    S.window->Render();
    S.interactor->ProcessEvents();
}

void zenslam::application::draw_viz_controls()
{
    ImGui::Text("Hello Metal!");
    ImGui::Separator();
    
    // Get current system state
    frame::system system { };
    {
        std::lock_guard lock { _mutex };
        system = _system;
    }

    // Current Frame Information
    ImGui::Text("Current Frame Info");
    ImGui::Spacing();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 6.0f));
    ImGui::BeginChild("frame_info_section", ImVec2(0, 200.0f), true);

    // Pose Information
    if (!std::isnan(system[1].timestamp))
    {
        const auto& pose = system[1].pose;
        const auto& translation = pose.translation();
        
        // Extract Euler angles from rotation matrix (returns [roll, pitch, yaw] in radians)
        const auto euler_rad = zenslam::utils::matrix_to_euler(pose.rotation());
        
        // Convert to degrees
        const auto euler_deg = euler_rad * (180.0 / CV_PI);
        
        ImGui::Text("Pose:");
        ImGui::Text("  Position [x, y, z]: [%.3f, %.3f, %.3f] m", 
                    translation[0], translation[1], translation[2]);
        ImGui::Text("  Euler [roll, pitch, yaw]: [%.2f, %.2f, %.2f] deg", 
                    euler_deg[0], euler_deg[1], euler_deg[2]);
        ImGui::Spacing();
    }

    // IMU Data
    if (!system[1].imu_data.empty())
    {
        const auto& imu = system[1].imu_data.back(); // Show latest IMU measurement
        
        ImGui::Text("IMU Data (%zu samples):", system[1].imu_data.size());
        ImGui::Text("  Angular Vel [wx, wy, wz]: [%.3f, %.3f, %.3f] rad/s",
                    imu.gyr[0], imu.gyr[1], imu.gyr[2]);
        ImGui::Text("  Linear Acc  [ax, ay, az]: [%.3f, %.3f, %.3f] m/s\u00b2",
                    imu.acc[0], imu.acc[1], imu.acc[2]);
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

    ImGui::Checkbox("Show Keypoints", &_options.slam.show_keypoints.value());
    ImGui::Checkbox("Show Keylines", &_options.slam.show_keylines.value());
    ImGui::SliderFloat("Point Cloud Opacity", &_point_cloud_opacity, 0.0f, 1.0f, "%.2f");

    // Color picker for keylines (single keyline color)
    const auto& s = _options.slam.keyline_single_color; // B, G, R
    ImVec4      color_rgba(static_cast<float>(s.value()[2]) / 255.0f,
                           // R
                           static_cast<float>(s.value()[1]) / 255.0f,
                           // G
                           static_cast<float>(s.value()[0]) / 255.0f,
                           // B
                           1.0f);

    if (ImGui::ColorEdit3("Keyline Color", reinterpret_cast<float*>(&color_rgba)))
    {
        const auto r                       = static_cast<int>(std::round(color_rgba.x * 255.0f));
        const auto g                       = static_cast<int>(std::round(color_rgba.y * 255.0f));
        const auto b                       = static_cast<int>(std::round(color_rgba.z * 255.0f));
        _options.slam.keyline_single_color = cv::Scalar(b, g, r);
    }

    ImGui::EndChild();
    ImGui::PopStyleVar(2);

    // Frame Duration Plots
    ImGui::Separator();
    ImGui::Text("Frame Durations");
    ImGui::Spacing();

    ImPlot::CreateContext();

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
        ImPlot::PlotLine("Processing",
                         _time_history.data(),
                         _processing_history.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Tracking",
                         _time_history.data(),
                         _tracking_history.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Estimation",
                         _time_history.data(),
                         _estimation_history.data(),
                         static_cast<int>(_time_history.size()));
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
        ImPlot::PlotLine("Tracked L",
                         _time_history.data(),
                         _point_history.features_l_tracked.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Tracked R",
                         _time_history.data(),
                         _point_history.features_r_tracked.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("New L",
                         _time_history.data(),
                         _point_history.features_l_new.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("New R",
                         _time_history.data(),
                         _point_history.features_r_new.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("KP Total",
                         _time_history.data(),
                         _point_history.features_total.data(),
                         static_cast<int>(_time_history.size()));

        // Matches & 3D
        ImPlot::PlotLine("Matches",
                         _time_history.data(),
                         _point_history.matches_stereo.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Triangulated",
                         _time_history.data(),
                         _point_history.triangulated_3d.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Map Points",
                         _time_history.data(),
                         _point_history.map_total.data(),
                         static_cast<int>(_time_history.size()));

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
        ImPlot::PlotLine("Tracked L",
                         _time_history.data(),
                         _line_history.features_l_tracked.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Tracked R",
                         _time_history.data(),
                         _line_history.features_r_tracked.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("New L",
                         _time_history.data(),
                         _line_history.features_l_new.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("New R",
                         _time_history.data(),
                         _line_history.features_r_new.data(),
                         static_cast<int>(_time_history.size()));

        // Matches & 3D
        ImPlot::PlotLine("KL Matches",
                         _time_history.data(),
                         _line_history.matches_stereo.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Lines 3D",
                         _time_history.data(),
                         _line_history.triangulated_3d.data(),
                         static_cast<int>(_time_history.size()));
        ImPlot::PlotLine("Map Lines",
                         _time_history.data(),
                         _line_history.map_total.data(),
                         static_cast<int>(_time_history.size()));

        ImPlot::EndPlot();
    }
}
