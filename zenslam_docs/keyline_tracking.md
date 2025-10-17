# Keyline Tracking with KLT

## Overview

This document describes the implementation of keyline tracking using the Kanade-Lucas-Tomasi (KLT) optical flow tracker with forward-backward error checking for robust track validation.

## Implementation

### Function Signature

```cpp
auto track_keylines
(
    const std::vector<cv::Mat> &     pyramid_0,
    const std::vector<cv::Mat> &     pyramid_1,
    const std::map<size_t, keyline> &keylines_map_0,
    const class options::slam &      options
) -> std::vector<keyline>;
```

### Algorithm

The keyline tracking algorithm works as follows:

#### 1. Endpoint Extraction
- Extract the start point `(startPointX, startPointY)` and end point `(endPointX, endPointY)` from each keyline
- Store all keylines in a vector for processing

#### 2. Forward Tracking
- Track the **start points** from frame 0 to frame 1 using `cv::calcOpticalFlowPyrLK`
- Track the **end points** from frame 0 to frame 1 using `cv::calcOpticalFlowPyrLK`
- Both tracks use the same KLT parameters:
  - Window size from `options.klt_window_size`
  - Maximum pyramid level from `options.klt_max_level`
  - Termination criteria: 99 iterations or 0.001 epsilon
  - `OPTFLOW_LK_GET_MIN_EIGENVALS` flag for better tracking quality

#### 3. Backward Tracking
- Track the **start points** from frame 1 back to frame 0
- Track the **end points** from frame 1 back to frame 0
- Uses the same KLT parameters as forward tracking

#### 4. Forward-Backward Error Checking
For each keyline, the algorithm:
1. Checks if both endpoints were successfully tracked in both directions (all four status flags must be true)
2. Computes the forward-backward error for each endpoint:
   - `fb_error_start = ||start_point_0_back - start_point_0||`
   - `fb_error_end = ||end_point_0_back - end_point_0||`
3. Accepts the track only if **both** endpoint errors are below `options.klt_threshold`

#### 5. Keyline Update
For successfully tracked keylines, the algorithm updates:
- **Endpoints**: `startPointX`, `startPointY`, `endPointX`, `endPointY`
- **Midpoint**: `pt.x = (startPointX + endPointX) / 2`, `pt.y = (startPointY + endPointY) / 2`
- **Length**: `lineLength = sqrt(dx² + dy²)`
- **Angle**: `angle = atan2(dy, dx) * 180 / π`

## Advantages

### 1. **Robustness Through Forward-Backward Checking**
- Eliminates tracks where optical flow "drifts" or loses the feature
- Ensures temporal consistency by verifying the track can be reversed
- Typical forward-backward threshold: 0.5-2.0 pixels

### 2. **Endpoint-Based Tracking**
- More efficient than tracking the entire line
- Preserves line geometry through endpoint constraints
- Naturally handles rotation and scale changes of the line segment

### 3. **Pyramid-Based Tracking**
- Uses image pyramids for multi-scale tracking
- Handles larger motions between frames
- More robust to illumination changes

## Usage Example

```cpp
// Given two consecutive frames with image pyramids
std::vector<cv::Mat> pyramid_0, pyramid_1;
std::map<size_t, keyline> keylines_map_0;  // Detected in frame 0

// Track keylines from frame 0 to frame 1
auto tracked_keylines = zenslam::utils::track_keylines(
    pyramid_0,
    pyramid_1,
    keylines_map_0,
    slam_options
);

// tracked_keylines now contains only successfully tracked keylines
// with updated positions in frame 1
```

## Parameters

The function uses the following parameters from `options::slam`:

- **`klt_window_size`**: Size of the search window at each pyramid level (typical: 21x21)
- **`klt_max_level`**: Maximum pyramid level (0 = no pyramid, typical: 3-5)
- **`klt_threshold`**: Forward-backward error threshold in pixels (typical: 0.5-2.0)

## Performance Considerations

- **Computational Cost**: 4 optical flow computations per keyline (2 forward + 2 backward)
- **Track Retention**: Typically 60-80% of keylines are successfully tracked between consecutive frames
- **Rejection Rate**: Forward-backward checking typically rejects 10-30% of initially successful forward tracks

## References

1. Kalal, Z., Mikolajczyk, K., & Matas, J. (2010). "Forward-backward error: Automatic detection of tracking failures." ICPR.
2. Lucas, B. D., & Kanade, T. (1981). "An iterative image registration technique with an application to stereo vision." IJCAI.
3. Bouguet, J. Y. (2001). "Pyramidal implementation of the affine Lucas Kanade feature tracker." Intel Corporation.
