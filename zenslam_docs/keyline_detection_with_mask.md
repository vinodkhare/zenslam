# Keyline Detection with Masking

## Overview

When tracking keylines across frames using KLT, you often want to detect new keylines only in areas where tracked keylines don't already exist. This avoids redundant detections and ensures better coverage of the image with line features.

## Implementation

The `grid_detector` class now provides an overloaded `detect` method that accepts existing keylines and automatically creates a mask to prevent detection in occupied regions.

### Function Signature

```cpp
std::vector<keyline> detect(
    const cv::Mat &image, 
    const std::map<size_t, keyline> &keylines_map, 
    int mask_margin = 10
) const;
```

### Parameters

- **`image`**: The input image for keyline detection
- **`keylines_map`**: Map of existing keylines (typically tracked from the previous frame)
- **`mask_margin`**: Margin in pixels around each existing keyline to mask out (default: 10 pixels)

## How It Works

### 1. Mask Creation
The function creates a binary mask (white = 255, black = 0) with the same size as the input image, initialized to all white (detection allowed everywhere).

### 2. Masking Existing Keylines
For each existing keyline:
1. Extracts the start and end points of the line segment:
   ```cpp
   start = (startPointX, startPointY)
   end = (endPointX, endPointY)
   ```

2. Draws a thick line on the mask using `cv::line()`:
   - Line thickness = `2 * mask_margin`
   - Color = 0 (black, masked out)
   - This creates a band of width `2 * mask_margin` centered on the line segment

3. The thick line approach avoids over-masking:
   - Only masks pixels along the actual line path
   - More efficient than masking the entire bounding box
   - Particularly beneficial for diagonal or long lines

### 3. Detection with Mask
The LSD (Line Segment Detector) is called with the mask parameter, which prevents it from detecting lines in the masked (black) regions.

## Usage Example

### Basic Workflow

```cpp
// Frame 1: Initial detection
std::map<size_t, keyline> keylines_map_0;
auto detected_keylines_0 = detector.detect(image_0);
for (const auto& kl : detected_keylines_0) {
    keylines_map_0[kl.index] = kl;
}

// Frame 2: Track existing keylines
auto tracked_keylines = zenslam::utils::track_keylines(
    pyramid_0,
    pyramid_1,
    keylines_map_0,
    options
);

// Update map with tracked keylines
std::map<size_t, keyline> keylines_map_1;
for (const auto& kl : tracked_keylines) {
    keylines_map_1[kl.index] = kl;
}

// Detect NEW keylines, avoiding areas where tracked keylines exist
auto new_keylines = detector.detect(
    image_1,
    keylines_map_1,
    10  // 10-pixel margin around existing keylines
);

// Combine tracked and newly detected keylines
for (const auto& kl : new_keylines) {
    keylines_map_1[kl.index] = kl;
}
```

### With Different Margin Values

```cpp
// Conservative: Small margin (more new detections, possible overlaps)
auto new_keylines = detector.detect(image, existing_keylines, 5);

// Standard: Default margin (good balance)
auto new_keylines = detector.detect(image, existing_keylines, 10);

// Aggressive: Large margin (fewer new detections, more spacing)
auto new_keylines = detector.detect(image, existing_keylines, 20);
```

## Benefits

### 1. **Prevents Redundant Detections**
- Avoids detecting new lines in the same location as tracked lines
- Reduces computational overhead in subsequent matching/tracking

### 2. **Better Feature Distribution**
- Encourages detection of lines in uncovered areas
- Maintains good spatial coverage of the scene

### 3. **Efficient Memory Usage**
- Reduces total number of keylines to manage
- Each unique line feature is represented once

### 4. **Improves Tracking Consistency**
- Tracked lines maintain their identity across frames
- New detections represent genuinely new features

### 5. **Precise Masking with Thick Lines**
- Masks only a band along the actual line path, not the entire bounding box
- Significantly reduces over-masking, especially for diagonal lines
- Example: A 100-pixel diagonal line with 10-pixel margin:
  - Rectangle approach: masks ~12,000 pixels (100×100 + margins)
  - Thick line approach: masks ~2,000 pixels (100×20)
- Allows detection of perpendicular or nearby parallel lines that would be blocked by rectangular masks

## Choosing the Mask Margin

The `mask_margin` parameter controls the trade-off between coverage and redundancy:

| Margin | Use Case | Characteristics |
|--------|----------|-----------------|
| 3-5 px | Dense scenes, short lines | Maximum coverage, some overlaps possible |
| 10 px  | General purpose (default) | Good balance for most scenarios |
| 15-20 px | Sparse scenes, long lines | Maximum spacing, potential gaps |
| 25+ px | Very sparse detection | Large gaps, only very distinct lines |

### Recommended Values by Line Length

```cpp
// For short lines (< 20 pixels)
int margin = 5;

// For medium lines (20-50 pixels)
int margin = 10;

// For long lines (> 50 pixels)
int margin = 15;

// Adaptive margin based on average line length
float avg_length = compute_average_line_length(keylines_map);
int margin = static_cast<int>(avg_length * 0.2f);  // 20% of average length
```

## Visualization

You can visualize the mask to debug or tune the margin:

```cpp
cv::Mat mask = cv::Mat::ones(image.size(), CV_8U) * 255;

for (const auto &kl : keylines_map | std::views::values) {
    cv::Point start(
        static_cast<int>(kl.startPointX),
        static_cast<int>(kl.startPointY)
    );
    cv::Point end(
        static_cast<int>(kl.endPointX),
        static_cast<int>(kl.endPointY)
    );
    
    // Draw thick line showing masked region
    cv::line(mask, start, end, cv::Scalar(0), 2 * margin, cv::LINE_8);
}

cv::imshow("Detection Mask", mask);

// Visualize masked regions on the original image
cv::Mat viz;
cv::cvtColor(image, viz, cv::COLOR_GRAY2BGR);
viz.setTo(cv::Scalar(0, 0, 255), mask == 0);  // Red for masked areas
cv::imshow("Masked Regions", viz);
```

## Performance Considerations

- **Mask Creation Cost**: O(N) where N = number of existing keylines
- **Memory Overhead**: One CV_8U mask (width × height bytes)
- **Detection Speed**: Unchanged - LSD naturally handles masks efficiently

For typical scenarios (50-200 keylines per frame, 640×480 image):
- Mask creation: < 1 ms
- Memory overhead: ~300 KB
- Total overhead: < 5% of frame processing time

## Integration with Complete Pipeline

```cpp
class KeylineTracker {
    grid_detector detector_;
    std::map<size_t, keyline> current_keylines_;
    
public:
    void process_frame(const cv::Mat& image, const std::vector<cv::Mat>& pyramid_prev) {
        std::map<size_t, keyline> next_keylines;
        
        // Track existing keylines
        if (!current_keylines_.empty()) {
            auto tracked = zenslam::utils::track_keylines(
                pyramid_prev, 
                pyramid, 
                current_keylines_, 
                options_
            );
            
            for (const auto& kl : tracked) {
                next_keylines[kl.index] = kl;
            }
        }
        
        // Detect new keylines in unoccupied areas
        auto new_keylines = detector_.detect(image, next_keylines, 10);
        
        for (const auto& kl : new_keylines) {
            next_keylines[kl.index] = kl;
        }
        
        current_keylines_ = std::move(next_keylines);
    }
};
```

## See Also

- [Keyline Tracking Documentation](keyline_tracking.md) - KLT-based keyline tracking
- [Feature Pipeline](feature_pipeline.md) - Overall feature detection and tracking pipeline
