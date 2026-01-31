# Quick Reference: Statistics Fields

## Timing Fields (seconds)

| Field | Description |
|-------|-------------|
| `t_wait` | Time waiting for frame in queue |
| `t_preprocessing` | Image preprocessing (CLAHE, rectification, pyramids) |
| `t_tracking` | Total tracking time (KLT + detection + matching) |
| `t_estimation` | Pose estimation time |
| `t_total` | Total frame processing time |
| `t_detection_left` | Left camera feature detection |
| `t_detection_right` | Right camera feature detection |
| `t_klt` | KLT optical flow tracking |
| `t_matching` | Stereo feature matching |
| `t_triangulation` | 3D point triangulation |

## Feature Count Fields

| Field | Description |
|-------|-------------|
| `n_keypoints_l` | Total keypoints in left camera |
| `n_keypoints_r` | Total keypoints in right camera |
| `n_tracked_l` | Keypoints tracked from previous frame (left) |
| `n_tracked_r` | Keypoints tracked from previous frame (right) |
| `n_new_l` | Newly detected keypoints (left) |
| `n_new_r` | Newly detected keypoints (right) |
| `n_matches` | Stereo matches between left/right |
| `n_triangulated` | Successfully triangulated 3D points |
| `n_3d3d` | 3D-3D correspondences for pose estimation |
| `n_3d2d` | 3D-2D correspondences for pose estimation |
| `n_2d2d` | 2D-2D correspondences for pose estimation |
| `n_3d3d_inliers` | RANSAC inliers for 3D-3D pose |
| `n_3d2d_inliers` | RANSAC inliers for 3D-2D pose |
| `n_2d2d_inliers` | RANSAC inliers for 2D-2D pose |

## Quality Metrics

| Field | Description |
|-------|-------------|
| `klt_error_mean` | Mean KLT tracking error (pixels) |
| `klt_error_std` | Std dev of KLT tracking error |
| `klt_success_rate` | Fraction of features successfully tracked (0-1) |
| `match_distance_mean` | Mean descriptor distance for matches |
| `match_distance_std` | Std dev of descriptor distances |
| `epipolar_error_mean` | Mean epipolar constraint error (pixels) |
| `fundamental_inliers` | Inliers to fundamental matrix in matching |
| `response_mean_l` | Mean feature response strength (left) |
| `response_mean_r` | Mean feature response strength (right) |
| `response_std_l` | Std dev of feature responses (left) |
| `response_std_r` | Std dev of feature responses (right) |
| `grid_occupancy_l` | Fraction of grid cells occupied (left, 0-1) |
| `grid_occupancy_r` | Fraction of grid cells occupied (right, 0-1) |
| `track_age_mean` | Mean lifetime of feature tracks (frames) |
| `track_age_max` | Maximum track lifetime (frames) |

## Configuration Fields

| Field | Description | Possible Values |
|-------|-------------|----------------|
| `detector_type` | Feature detector used | FAST, ORB, SIFT |
| `descriptor_type` | Feature descriptor used | ORB, SIFT, FREAK |
| `matcher_type` | Feature matcher used | BRUTE, KNN, FLANN |

## Typical Ranges (for reference)

Based on typical SLAM scenarios:

- **Good KLT success rate**: > 40%
- **Minimum matches for robustness**: > 30
- **Minimum inliers for pose**: > 15-20
- **Real-time performance**: t_total < 33ms (30 FPS) or < 50ms (20 FPS)
- **Response values**: Depends on detector (FAST: 20-100, ORB: 30-80, SIFT: 0-1)
- **Grid occupancy**: 30-70% is typical for well-distributed features

## Warning Thresholds

Consider investigating frames with:

- `klt_success_rate < 0.3` (30%)
- `n_matches < 20`
- `n_triangulated < 10`
- `n_3d3d_inliers + n_3d2d_inliers < 10`
- `t_total > 0.05` (50ms for 20 FPS)
