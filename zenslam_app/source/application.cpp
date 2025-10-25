#include "application.h"

#include <imgui.h>

#include <opencv2/highgui.hpp>

#include <zenslam/utils.h>

#include <utility>

#include "zenslam/utils_opencv.h"

// VTK includes (kept in .cpp)
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkInteractorStyleTrackballCamera.h>

zenslam::application::application(options options) :
    _options{std::move(options)},
    _show_keypoints{_options.slam.show_keypoints},
    _show_keylines{_options.slam.show_keylines}
{
    _slam_thread.on_frame += [this](const frame::system& frame)
    {
        std::lock_guard lock{_mutex};
        _system = frame;
    };

    _reader_thread.on_frame += [this](const frame::sensor& frame)
    {
        _slam_thread.enqueue(frame);
    };
}

void zenslam::application::render()
{
    frame::system system{};
    {
        std::lock_guard lock{_mutex};
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
    return !system[0].undistorted[0].empty() &&
        !system[1].undistorted[0].empty() &&
        !system[1].keypoints[0].empty();
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
        const auto& P = pose.matrix; // 4x4 double
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                m->SetElement(r, c, P(r, c));
        return m;
    }
}

// Local VTK scene container
struct zenslam::application::SceneVTK
{
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> window;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;

    // Point cloud
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkPolyData> pointsPoly;
    vtkSmartPointer<vtkVertexGlyphFilter> pointsGlyph;
    vtkSmartPointer<vtkPolyDataMapper> pointsMapper;
    vtkSmartPointer<vtkActor> pointsActor;

    // Lines
    vtkSmartPointer<vtkPoints> linePoints;
    vtkSmartPointer<vtkCellArray> lineCells;
    vtkSmartPointer<vtkPolyData> linesPoly;
    vtkSmartPointer<vtkPolyDataMapper> linesMapper;
    vtkSmartPointer<vtkActor> linesActor;

    // Axes for camera poses
    vtkSmartPointer<vtkAxesActor> axesCam;
    vtkSmartPointer<vtkAxesActor> axesCamGt;
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

        S.renderer = vtkSmartPointer<vtkRenderer>::New();
        S.window = vtkSmartPointer<vtkRenderWindow>::New();
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

        S.axesCam = vtkSmartPointer<vtkAxesActor>::New();
        S.axesCamGt = vtkSmartPointer<vtkAxesActor>::New();
        S.axesCam->SetTotalLength(0.2, 0.2, 0.2);
        S.axesCamGt->SetTotalLength(0.2, 0.2, 0.2);
        S.renderer->AddActor(S.axesCam);
        S.renderer->AddActor(S.axesCamGt);

        // Points pipeline
        S.points = vtkSmartPointer<vtkPoints>::New();
        S.pointsPoly = vtkSmartPointer<vtkPolyData>::New();
        S.pointsGlyph = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        S.pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.pointsActor = vtkSmartPointer<vtkActor>::New();

        S.pointsPoly->SetPoints(S.points);
        S.pointsGlyph->SetInputData(S.pointsPoly);
        S.pointsGlyph->Update();
        S.pointsMapper->SetInputConnection(S.pointsGlyph->GetOutputPort());
        S.pointsActor->SetMapper(S.pointsMapper);
        S.pointsActor->GetProperty()->SetPointSize(4.0);
        S.renderer->AddActor(S.pointsActor);

        // Lines pipeline
        S.linePoints = vtkSmartPointer<vtkPoints>::New();
        S.lineCells = vtkSmartPointer<vtkCellArray>::New();
        S.linesPoly = vtkSmartPointer<vtkPolyData>::New();
        S.linesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.linesActor = vtkSmartPointer<vtkActor>::New();

        S.linesPoly->SetPoints(S.linePoints);
        S.linesPoly->SetLines(S.lineCells);
        S.linesMapper->SetInputData(S.linesPoly);
        S.linesActor->SetMapper(S.linesMapper);
        S.linesActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // green
        S.linesActor->GetProperty()->SetLineWidth(1.0);
        S.renderer->AddActor(S.linesActor);
    }

    const auto& S = *_vtk;

    // Update camera axes transforms
    {
        const vtkNew<vtkTransform> tCam;
        tCam->SetMatrix(toVtkMatrix(system[1].pose));
        S.axesCam->SetUserTransform(tCam);

        const vtkNew<vtkTransform> tCamGt;
        tCamGt->SetMatrix(toVtkMatrix(system[1].pose_gt));
        S.axesCamGt->SetUserTransform(tCamGt);
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

    // Render one frame; process user events without blocking
    S.window->Render();
    S.interactor->ProcessEvents();
}

void zenslam::application::draw_viz_controls()
{
    ImGui::Text("Hello Metal!");
    ImGui::Separator();
    ImGui::Text("Visualization Options");
    ImGui::Spacing();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 6.0f));
    ImGui::BeginChild("viz_section", ImVec2(0, 160.0f), true);

    ImGui::Checkbox("Show Keypoints", &_options.slam.show_keypoints);
    ImGui::Checkbox("Show Keylines", &_options.slam.show_keylines);

    // Color picker for keylines (single keyline color)
    const auto& s = _options.slam.keyline_single_color; // B, G, R
    ImVec4 color_rgba(
        static_cast<float>(s[2]) / 255.0f, // R
        static_cast<float>(s[1]) / 255.0f, // G
        static_cast<float>(s[0]) / 255.0f, // B
        1.0f
        );

    if (ImGui::ColorEdit3("Keyline Color", reinterpret_cast<float*>(&color_rgba)))
    {
        const auto r = static_cast<int>(std::round(color_rgba.x * 255.0f));
        const auto g = static_cast<int>(std::round(color_rgba.y * 255.0f));
        const auto b = static_cast<int>(std::round(color_rgba.z * 255.0f));
        _options.slam.keyline_single_color = cv::Scalar(b, g, r);
    }

    ImGui::EndChild();
    ImGui::PopStyleVar(2);
}
