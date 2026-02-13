# Matcher Options

ZenSLAM supports three matching strategies for stereo keypoint matching: brute-force with cross-check, k-nearest neighbors (kNN) with Lowe's ratio test, and FLANN-based matching for fast approximate nearest neighbor search.

## Configuration Options

### `matcher` (enum)
Selects the matching algorithm:
- **`brute`** (default): Uses brute-force matching with mutual cross-check. This is the most conservative approach, keeping only mutual nearest neighbors. Provides high precision but may have lower recall.
- **`knn`**: Uses k-nearest neighbors matching (k=2) with Lowe's ratio test. This typically provides 1.5–3× more matches than brute-force while maintaining good quality through ratio-based filtering.
- **`flann`**: Uses FLANN (Fast Library for Approximate Nearest Neighbors) with ratio test. Significantly faster than BFMatcher for large descriptor sets, especially with float descriptors (SIFT, SURF). Automatically selects LSH for binary descriptors (ORB, BRISK) or KDTree for float descriptors.

### `matcher_ratio` (float)
Ratio test threshold for kNN and FLANN matching (range: 0.0–1.0, default: 0.8).

Only applies when `matcher: knn` or `matcher: flann`. A match is kept if the distance to the best match is less than `matcher_ratio` times the distance to the second-best match. This is Lowe's ratio test, which effectively filters out ambiguous matches.

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
  matcher: "knn"
  matcher_ratio: 0.8
  
  # Or use FLANN for faster matching (especially for float descriptors)
  # matcher: "flann"
  # matcher_ratio: 0.8
  
  # Or use brute-force with cross-check (default)
  # matcher: "brute"
  # matcher_ratio is ignored for brute mode
```

### Command Line

```bash
```bash
./zenslam_app \
  --matcher flann \
  --matcher-ratio 0.8 \
  # ... other arguments ...
```

### Keep Original Behavior
```yaml
slam:
  matcher: "brute"  # or omit (default)
```

## When to Use Each Mode

### Use `brute` when:
- You need maximum matching precision
- Your scene has repetitive textures or ambiguous features
- You have plenty of detected features and can afford to be selective
- Your calibration is excellent and geometric filtering is very reliable
- Descriptor set is small (< 500 features per image)

### Use `knn` when:
- You want to increase the number of stereo matches
- You're experiencing too few triangulated points
- Your feature detector is conservative (low feature count)
- Your scene has good texture and low ambiguity
- You want better coverage across the image
- Descriptor set is small to medium (< 1000 features)

### Use `flann` when:
- You have **many features** (> 1000 per image) and need speed
- Using **float descriptors** (SIFT, SURF) where FLANN excels
- Real-time performance is critical
- You want good recall with acceptable precision trade-off
- Memory usage is a concern (FLANN uses less memory than BF for large sets)

## FLANN Advantages

**Speed**: FLANN is 5–50× faster than brute-force for large descriptor sets
- **Binary descriptors (ORB)**: 5–10× faster with LSH indexing
- **Float descriptors (SIFT)**: 10–50× faster with KDTree indexing

**Memory**: Lower memory footprint for large feature sets

**Quality**: Similar recall to kNN with proper ratio threshold

**Scalability**: Performance advantage grows with descriptor count

### Performance Comparison (1000 descriptors)

| Matcher | Speed | Memory | Match Quality |
|---------|-------|--------|---------------|
| brute   | 1×    | High   | Highest precision, lower recall |
| knn     | 1×    | High   | High precision, good recall |
| flann   | 8×    | Medium | Good precision, good recall |

*Note: FLANN advantage increases dramatically with > 2000 descriptors*

## Implementation Details

All three matchers apply the same epipolar filtering after descriptor matching:

1. **brute mode**: BFMatcher with `crossCheck=true` → epipolar filter
2. **knn mode**: BFMatcher with `crossCheck=false` → knnMatch(k=2) → ratio test → epipolar filter
3. **flann mode**: FlannBasedMatcher with LSH (binary) or KDTree (float) → knnMatch(k=2) → ratio test → epipolar filter

### FLANN Index Parameters

**For binary descriptors (ORB, BRISK, AKAZE)**:
- Algorithm: LSH (Locality Sensitive Hashing)
- Parameters: `LshIndexParams(12, 20, 2)`
  - 12 hash tables
  - 20 key bits
  - 2 multi-probe level

**For float descriptors (SIFT, SURF)**:
- Algorithm: KDTree
- Parameters: `KDTreeIndexParams(4)`
  - 4 randomized trees for better accuracy

The kNN+ratio approach (both knn and flann) often yields more matches because:
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

1. **Start with your descriptor type**:
   - Binary (ORB, BRISK): Try `knn` first, use `flann` if > 1000 features
   - Float (SIFT, SURF): Use `flann` for best speed/quality trade-off

2. **Monitor triangulation quality**: Check reprojection errors and epipolar angles to ensure matches are geometrically consistent.

3. **Adjust ratio for your descriptor**:
   - Binary descriptors: 0.75–0.82
   - Float descriptors: 0.7–0.8
   
4. **Combine with epipolar threshold**: If using looser matching (`knn` or `flann` with higher ratio), consider slightly tightening `epipolar_threshold` to maintain geometric consistency.

5. **Visualize results**: Use the counts plot to monitor `matches` and `triangulated` counts across frames.

6. **Profile if speed matters**: For real-time applications with many features, FLANN can be the difference between 15 FPS and 30 FPS.

## Example Comparison

For a typical indoor scene with ORB descriptors (500 features per image):

| Matcher | Ratio | Matches | Triangulated | Speed | Precision |
|---------|-------|---------|--------------|-------|-----------|
| brute   | N/A   | ~150    | ~120         | 1×    | Very High |
| knn     | 0.75  | ~280    | ~230         | 1×    | High      |
| knn     | 0.80  | ~350    | ~290         | 1×    | Good      |
| flann   | 0.75  | ~270    | ~220         | 7×    | High      |
| flann   | 0.80  | ~340    | ~280         | 7×    | Good      |

For a large outdoor scene with SIFT descriptors (2000 features per image):

| Matcher | Ratio | Matches | Triangulated | Speed  | Precision |
|---------|-------|---------|--------------|--------|-----------|
| brute   | N/A   | ~600    | ~480         | 1×     | Very High |
| knn     | 0.75  | ~1100   | ~900         | 1×     | High      |
| flann   | 0.75  | ~1050   | ~860         | 25×    | High      |
| flann   | 0.80  | ~1300   | ~1070        | 25×    | Good      |

*Note: Actual numbers vary by scene texture, calibration quality, and detector settings.*

## Practical Recommendations

### Real-time SLAM (target: 30 Hz)
```yaml
slam:
  descriptor: "ORB"
  matcher: "flann"
  matcher_ratio: 0.8
```

### High-quality mapping (offline)
```yaml
slam:
  descriptor: "SIFT"  # or SURF
  matcher: "flann"
  matcher_ratio: 0.75
```

### Conservative/baseline
```yaml
slam:
  descriptor: "ORB"
  matcher: "brute"
```

### Maximum recall (feature-poor scenes)
```yaml
slam:
  matcher: "knn"  # or flann
  matcher_ratio: 0.85
  epipolar_threshold: 1.5  # slightly tighter to compensate
```

## See Also

- `epipolar_threshold`: Geometric filter threshold (pixels)
- `triangulation_reprojection_threshold`: Post-triangulation quality filter
- Feature detector options: `feature`, `descriptor`, `fast_threshold`
