# Keyline Feature Detection and Tracking

Comprehensive guide to line feature (keyline) detection, tracking, and masking in ZenSLAM.

## Overview

Keylines (line segments) complement point features by providing geometric constraints from linear structures in the environment (edges, boundaries, architectural features). ZenSLAM implements:

1. **LSD Detection** with grid-based spatial distribution
2. **KLT Tracking** for temporal correspondence  
3. **Intelligent Masking** to avoid redundant detections

## Detection with Masking

### Purpose

When tracking keylines across frames, detect new keylines only in unoccupied regions to avoid redundancy and ensure better image coverage.

### Implementation

```cpp
std::vector<keyline> detect(
    const cv::Mat &image, 
    const std::map<size_t, keyline> &keylines_map, 
    int mask_margin = 10
) const;
```

**Parameters:**
- `image` - Input image for keyline detection
- `keylines_map` - Existing keylines (tracked from previous frame)
- `mask_margin` - Margin in pixels around existing keylines (default: 10)

### Masking Algorithm

1. **Create binary mask** (white = 255) initialized to allow detection everywhere
2. **For each existing keyline:**
   - Extract start `(startPointX, startPointY)` and end `(endPointX, endPointY)` points
   - Draw thick line with `cv::line()`:
     - Thickness: `2 * mask_margin`
     - Color: 0 (black, masked)
3. **Run LSD** with mask to prevent detection in masked regions

### Masking Strategy: Thick Line vs Rectangle

**Thick Line Approach** (current implementation):
- Masks only pixels along line path with margin
- More efficient for diagonal lines
- Allows detection of perpendicular features

**Efficiency Comparison** (100-pixel line, 10-pixel margin):

| Line Angle | Rectangle | Thick Line | Reduction |
|------------|-----------|------------|-----------|
| 0° (horizontal) | 2,400 px | 2,200 px | 8% |
| 45° (diagonal) | 14,400 px | 2,830 px | **80%** |
| 90° (vertical) | 2,400 px | 2,200 px | 8% |

**Key Insight:** Diagonal lines benefit most from thick line masking!

### Usage Example

```cpp
// Frame 0: Initial detection
std::map<size_t, keyline> keylines_map_0;
auto detected_keylines_0 = detector.detect(image_0);
for (const auto& kl : detected_keylines_0) {
    keylines_map_0[kl.index] = kl;
}

// Frame 1: Track existing keylines
auto tracked_keylines = utils::track_keylines(
    pyramid_0, pyramid_1, keylines_map_0, options
);

// Update map with tracked keylines
std::map<size_t, keyline> keylines_map_1;
for (const auto& kl : tracked_keylines) {
    keylines_map_1[kl.index] = kl;
}

// Detect NEW keylines, avoiding tracked regions
auto new_keylines = detector.detect(image_1, keylines_map_1, 10);

// Combine tracked + newly detected
for (const auto& kl : new_keylines) {
    keylines_map_1[kl.index] = kl;
}
```

## Keyline Tracking with KLT

### Algorithm

Uses Kanade-Lucas-Tomasi optical flow with forward-backward error checking for robust validation.

```cpp
auto track_keylines(
    const std::vector<cv::Mat> & pyramid_0,
    const std::vector<cv::Mat> & pyramid_1,
    const std::map<size_t, keyline> & keylines_map_0,
    const options::slam & options
) -> std::vector<keyline>;
```

### Tracking Steps

#### 1. Endpoint Extraction
Extract start `(startPointX, startPointY)` and end `(endPointX, endPointY)` from each keyline.

#### 2. Forward Tracking
Track **start points** and **end points** from frame 0 → frame 1 using `cv::calcOpticalFlowPyrLK`:
- Window size: `options.klt_window_size`
- Max pyramid level: `options.klt_max_level`
- Termination: 99 iterations or 0.001 epsilon
- Flag: `OPTFLOW_LK_GET_MIN_EIGENVALS` for quality tracking

#### 3. Backward Tracking
Track points from frame 1 → frame 0 (back to origin) with same parameters.

#### 4. Forward-Backward Error Check
For each keyline:
1. Verify both endpoints tracked successfully in both directions (status flags true)
2. Compute FB errors:
   - `fb_error_start = ||start_point_0_back - start_point_0||`
   - `fb_error_end = ||end_point_0_back - end_point_0||`
3. Accept only if **both** errors < `options.klt_threshold`

#### 5. Keyline Update
For successfully tracked keylines:
- Update endpoints: `startPointX`, `startPointY`, `endPointX`, `endPointY`
- Recompute midpoint: `pt = (start + end) / 2`
- Recalculate length: `lineLength = ||end - start||`
- Recalculate angle: `angle = atan2(dy, dx) * 180 / π`

## Advantages

### Forward-Backward Checking
- **Robust to occlusions** - Detects if feature disappears
- **Drift detection** - Identifies accumulated tracking error
- **Outlier rejection** - Filters false correspondences

### Endpoint Tracking
- **Geometric consistency** - Preserves line segment structure
- **Detection of rotation/shearing** - Both endpoints may move differently
- **Better than midpoint only** - Full line geometry preserved

### Double-Threshold Validation
- **Strict requirement** - Both endpoints must satisfy error threshold
- **Eliminates partial tracks** - One good endpoint insufficient
- **Higher quality tracks** - Better for pose estimation

## Configuration

**Key Parameters:**
- `klt_window_size` - Search window (typically 21-31)
- `klt_max_level` - Pyramid levels (typically 3-5)
- `klt_threshold` - FB error threshold in pixels (typically 1.0-2.0)
- `mask_margin` - Detection mask margin (typically 10)

## Future Enhancements

- Line descriptor matching for loop closure
- Multi-scale line detection
- Line segment merging/splitting
- Integration with point-line bundle adjustment
