#include "vtk_scene_window.h"

// VTK includes (kept in .cpp to avoid leaking VTK headers)
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkCellArray.h>
#include <vtkImageImport.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLine.h>
#include <vtkMatrix4x4.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>

#include <opencv2/imgproc.hpp>

namespace zenslam { namespace
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
    struct vtk_scene_window::scene_vtk
    {
        vtkSmartPointer<vtkRenderer>               renderer;
        vtkSmartPointer<vtkRenderWindow>           window;
        vtkSmartPointer<vtkRenderWindowInteractor> interactor;

        // Point cloud
        vtkSmartPointer<vtkPoints>            points;
        vtkSmartPointer<vtkUnsignedCharArray> pointColors;
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

    // Ensure unique_ptr<scene_vtk> destruction sees a complete type
    void vtk_scene_window::scene_vtk_deleter::operator()(const scene_vtk* p) const { delete p; }

    vtk_scene_window::vtk_scene_window(const options&            options,
                                       gui_options&             gui_options,
                                       std::vector<cv::Point3d>& trajectory_estimated,
                                       std::vector<cv::Point3d>& trajectory_gt) :
        _options(options),
        _gui_options(gui_options),
        _trajectory_estimated(trajectory_estimated),
        _trajectory_gt(trajectory_gt)
    {
    }

    vtk_scene_window::~vtk_scene_window() = default;

    void vtk_scene_window::initialize()
    {
        if (_initialized)
            return;

        _scene.reset(new scene_vtk());
        auto& S = *_scene;

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

        // Gradient background: dark blue (bottom) to light blue (top)
        S.renderer->SetBackground(0.05, 0.1, 0.2);   // Dark blue
        S.renderer->SetBackground2(0.4, 0.6, 0.9);   // Light blue
        S.renderer->SetGradientBackground(true);

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
        S.points      = vtkSmartPointer<vtkPoints>::New();
        S.pointColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        S.pointColors->SetNumberOfComponents(3);
        S.pointColors->SetName("Colors");
        S.pointsPoly   = vtkSmartPointer<vtkPolyData>::New();
        S.pointsGlyph  = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        S.pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.pointsActor  = vtkSmartPointer<vtkActor>::New();

        S.pointsPoly->SetPoints(S.points);
        S.pointsPoly->GetPointData()->SetScalars(S.pointColors);
        S.pointsGlyph->SetInputData(S.pointsPoly);
        S.pointsGlyph->Update();
        S.pointsMapper->SetInputConnection(S.pointsGlyph->GetOutputPort());
        S.pointsActor->SetMapper(S.pointsMapper);
        S.pointsActor->GetProperty()->SetPointSize(_gui_options.point_size);
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
        S.trajectoryPoints = vtkSmartPointer<vtkPoints>::New();
        S.trajectoryCells  = vtkSmartPointer<vtkCellArray>::New();
        S.trajectoryPoly   = vtkSmartPointer<vtkPolyData>::New();
        S.trajectoryMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.trajectoryActor  = vtkSmartPointer<vtkActor>::New();

        S.trajectoryPoly->SetPoints(S.trajectoryPoints);
        S.trajectoryPoly->SetLines(S.trajectoryCells);
        S.trajectoryMapper->SetInputData(S.trajectoryPoly);
        S.trajectoryActor->SetMapper(S.trajectoryMapper);
        S.trajectoryActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // red for estimated
        S.trajectoryActor->GetProperty()->SetLineWidth(3.0);
        S.renderer->AddActor(S.trajectoryActor);

        // Trajectory pipeline (ground truth)
        S.trajectoryGtPoints = vtkSmartPointer<vtkPoints>::New();
        S.trajectoryGtCells  = vtkSmartPointer<vtkCellArray>::New();
        S.trajectoryGtPoly   = vtkSmartPointer<vtkPolyData>::New();
        S.trajectoryGtMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        S.trajectoryGtActor  = vtkSmartPointer<vtkActor>::New();

        S.trajectoryGtPoly->SetPoints(S.trajectoryGtPoints);
        S.trajectoryGtPoly->SetLines(S.trajectoryGtCells);
        S.trajectoryGtMapper->SetInputData(S.trajectoryGtPoly);
        S.trajectoryGtActor->SetMapper(S.trajectoryGtMapper);
        S.trajectoryGtActor->GetProperty()->SetColor(0.0, 1.0, 1.0); // cyan for ground truth
        S.trajectoryGtActor->GetProperty()->SetLineWidth(2.0);
        S.renderer->AddActor(S.trajectoryGtActor);

        _initialized = true;
    }

    void vtk_scene_window::render(const frame::system& system)
    {
        if (!_visible)
            return;

        if (!_initialized)
            initialize();

        const auto& S = *_scene;

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
            S.pointColors->Reset();

            for (const auto& i : system.points3d.values() | std::ranges::to<std::vector>())
            {
                // Add color (BGR to RGB conversion)
                const unsigned char rgb[3] = { i.color[2], i.color[1], i.color[0] }; // BGR to RGB

                S.points->InsertNextPoint(i.x, i.y, i.z);
                S.pointColors->InsertNextTypedTuple(rgb);
            }

            S.points->Modified();
            S.pointColors->Modified();
            S.pointsPoly->Modified();
            S.pointsGlyph->Update();
            S.pointsActor->GetProperty()->SetOpacity(_gui_options.point_cloud_opacity);
            S.pointsActor->GetProperty()->SetPointSize(_gui_options.point_size);
        }

        // Update or hide lines
        if (_options.slam->gui->show_keylines)
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

    void vtk_scene_window::set_background_color(double r1, double g1, double b1, 
                                                 double r2, double g2, double b2)
    {
        if (!_initialized || !_scene)
            return;

        auto& S = *_scene;

        // If second color not specified (negative values), use solid color
        if (r2 < 0.0 || g2 < 0.0 || b2 < 0.0)
        {
            S.renderer->SetBackground(r1, g1, b1);
            S.renderer->SetGradientBackground(false);
        }
        else
        {
            // Use gradient background
            S.renderer->SetBackground(r1, g1, b1);   // Bottom color
            S.renderer->SetBackground2(r2, g2, b2);  // Top color
            S.renderer->SetGradientBackground(true);
        }
    }
} // namespace zenslam
