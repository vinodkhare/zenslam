# Matcher Options

ZenSLAM now supports two matching strategies for stereo keypoint matching: brute-force with cross-check and k-nearest neighbors (kNN) with Lowe's ratio test.

## Configuration Options

### `matcher` (enum)
Selects the matching algorithm:
- **`BRUTE`** (default): Uses brute-force matching with mutual cross-check. This is the most conservative approach, keeping only mutual nearest neighbors. Provides high precision but may have lower recall.
- **`KNN`**: Uses k-nearest neighbors matching (k=2) with Lowe's ratio test. This typically provides 1.5–3× more matches than brute-force while maintaining good quality through ratio-based filtering.

### `matcher_ratio` (float)
Ratio test threshold for kNN matching (range: 0.0–1.0, default: 0.8).

Only applies when `matcher: KNN`. A match is kept if the distance to the best match is less than `matcher_ratio` times the distance to the second-best match. This is Lowe's ratio test, which effectively filters out ambiguous matches.

**Recommended values:**
- **0.75**: Conservative, higher precision, fewer matches
- **0.8**: Balanced (recommended starting point)
- **0.85**: More permissive, higher recall, slightly lower precision

## Usage

### YAML Configuration

```yaml
slam:
  # ... other options ...
  
  # Use kNN matching with ratio test
  matcher: "KNN"
  matcher_ratio: 0.8
  
  # Or use brute-force with cross-check (default)
  # matcher: "BRUTE"
  # matcher_ratio is ignored for BRUTE mode
```

### Command Line

```bash
./zenslam_app \
  --matcher KNN \
  --matcher-ratio 0.8 \
  # ... other arguments ...
```

## When to Use Each Mode

### Use `BRUTE` when:
- You need maximum matching precision
- Your scene has repetitive textures or ambiguous features
- You have plenty of detected features and can afford to be selective
- Your calibration is excellent and geometric filtering is very reliable

### Use `KNN` when:
- You want to increase the number of stereo matches
- You're experiencing too few triangulated points
- Your feature detector is conservative (low feature count)
- Your scene has good texture and low ambiguity
- You want better coverage across the image

## Implementation Details

Both matchers apply the same epipolar filtering after descriptor matching:
1. **BRUTE mode**: BFMatcher with `crossCheck=true` → epipolar filter
2. **KNN mode**: BFMatcher with `crossCheck=false` → knnMatch(k=2) → ratio test → epipolar filter

The kNN+ratio approach often yields more matches because:
- Cross-check is strict (requires mutual nearest neighbors)
- Ratio test disambiguates by comparing first and second-best matches
- Many true matches that fail cross-check pass the ratio test

## Tuning Tips

1. **Start with KNN and ratio=0.8**: This is a good baseline that typically increases matches while maintaining quality.

2. **Monitor triangulation quality**: Check reprojection errors and epipolar angles to ensure matches are geometrically consistent.

3. **Adjust ratio for your descriptor**:
   - Binary descriptors (ORB, BRISK): 0.75–0.82
   - Float descriptors (SIFT, SURF): 0.7–0.8
   
4. **Combine with epipolar threshold**: If using looser matching (`KNN` with higher ratio), consider slightly tightening `epipolar_threshold` to maintain geometric consistency.

5. **Visualize results**: Use the counts plot to monitor `matches` and `triangulated` counts across frames.

## Example Comparison

For a typical indoor scene with ORB descriptors:

| Matcher | Ratio | Matches | Triangulated | Precision |
|---------|-------|---------|--------------|-----------|
| BRUTE   | N/A   | ~150    | ~120         | Very High |
| KNN     | 0.75  | ~280    | ~230         | High      |
| KNN     | 0.80  | ~350    | ~290         | Good      |
| KNN     | 0.85  | ~420    | ~340         | Moderate  |

*Note: Actual numbers vary by scene texture, calibration quality, and detector settings.*

## See Also

- `epipolar_threshold`: Geometric filter threshold (pixels)
- `triangulation_reprojection_threshold`: Post-triangulation quality filter
- Feature detector options: `feature`, `descriptor`, `fast_threshold`
