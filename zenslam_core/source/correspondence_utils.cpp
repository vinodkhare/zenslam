#include "zenslam/correspondence_utils.h"

#include <ranges>
#include <vector>

#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/point3d.h"

auto zenslam::utils::correspondence_2d2d(
    const std::map<size_t, keypoint>& keypoints_0,
    const std::map<size_t, keypoint>& keypoints_1,
    std::vector<cv::Point2f>&         points2f_0,
    std::vector<cv::Point2f>&         points2f_1,
    std::vector<size_t>&              indices) -> void
{
    for (const auto& index_1 : keypoints_1 | std::views::keys)
    {
        if (keypoints_0.contains(index_1))
        {
            points2f_0.emplace_back(keypoints_0.at(index_1).pt);
            points2f_1.emplace_back(keypoints_1.at(index_1).pt);
            indices.emplace_back(index_1);
        }
    }
}

void zenslam::utils::correspondences_3d2d(
    const std::map<size_t, point3d>&  points,
    const std::map<size_t, keypoint>& keypoints,
    std::vector<cv::Point3d>&         points3d,
    std::vector<cv::Point2d>&         points2d,
    std::vector<size_t>&              indices)
{
    for (const auto& index : keypoints | std::views::keys)
    {
        if (points.contains(index))
        {
            points3d.emplace_back(points.at(index));
            points2d.emplace_back(keypoints.at(index).pt);
            indices.emplace_back(index);
        }
    }
}

void zenslam::utils::correspondences_3d3d(
    const std::map<size_t, point3d>& points_map_0,
    const std::map<size_t, point3d>& points_map_1,
    std::vector<cv::Point3d>&        points3d_0,
    std::vector<cv::Point3d>&        points3d_1,
    std::vector<size_t>&             indexes)
{
    for (const auto& index : points_map_1 | std::views::keys)
    {
        if (points_map_0.contains(index))
        {
            points3d_0.emplace_back(points_map_0.at(index));
            points3d_1.emplace_back(points_map_1.at(index));
            indexes.emplace_back(index);
        }
    }
}

void zenslam::utils::correspondences_3d2d_lines(
    const std::map<size_t, line3d>&  lines_map,
    const std::map<size_t, keyline>& keylines_map,
    std::vector<cv::Point3d>&        lines3d_p1,
    std::vector<cv::Point3d>&        lines3d_p2,
    std::vector<cv::Point2d>&        keylines2d_p1,
    std::vector<cv::Point2d>&        keylines2d_p2,
    std::vector<size_t>&             indices)
{
    for (const auto& index : keylines_map | std::views::keys)
    {
        if (lines_map.contains(index))
        {
            const auto& line    = lines_map.at(index);
            const auto& keyline = keylines_map.at(index);

            lines3d_p1.emplace_back(line[0]);
            lines3d_p2.emplace_back(line[1]);
            keylines2d_p1.emplace_back(keyline.startPointX, keyline.startPointY);
            keylines2d_p2.emplace_back(keyline.endPointX, keyline.endPointY);
            indices.emplace_back(index);
        }
    }
}

void zenslam::utils::correspondences_3d3d_lines(
    const std::map<size_t, line3d>& lines_map_0,
    const std::map<size_t, line3d>& lines_map_1,
    std::vector<cv::Point3d>&       lines3d_0_p1,
    std::vector<cv::Point3d>&       lines3d_0_p2,
    std::vector<cv::Point3d>&       lines3d_1_p1,
    std::vector<cv::Point3d>&       lines3d_1_p2,
    std::vector<size_t>&            indices)
{
    for (const auto& index : lines_map_1 | std::views::keys)
    {
        if (lines_map_0.contains(index))
        {
            const auto& line_0 = lines_map_0.at(index);
            const auto& line_1 = lines_map_1.at(index);

            lines3d_0_p1.emplace_back(line_0[0]);
            lines3d_0_p2.emplace_back(line_0[1]);
            lines3d_1_p1.emplace_back(line_1[0]);
            lines3d_1_p2.emplace_back(line_1[1]);
            indices.emplace_back(index);
        }
    }
}
