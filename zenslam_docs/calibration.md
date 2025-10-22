# Calibration & Camera Model

ZenSLAM parses Kalibr-style YAML stereo calibration files (e.g., TUM-VI style) to obtain:

- Intrinsics: `fx, fy, cx, cy`
- Distortion coefficients (equidistant or radial-tangential; future extension)
- Resolution
- Relative pose between cameras (`T_cn_cnm1`)

## Parsing

`calibration::parse(path, camera_name)` loads a node and populates a `calibration` object.

## Stored Fields

```
std::string camera_name
cv::Size    resolution
cv::Vec2d   focal_length   (fx, fy)
cv::Vec2d   principal_point (cx, cy)
std::vector<double> distortion_coefficients
cv::Affine3d pose_in_cam0  // Extrinsics wrt primary camera
```

## Derived Matrices

- Camera matrix: `K = [[fx,0,cx],[0,fy,cy],[0,0,1]]`
- Fundamental Matrix: `F = K2^{-T} [t]_x R K1^{-1}` using relative pose between two cameras.
- Projection Matrix: `P = K [R|t]` (via minor extraction from pose affine matrix).

## Stereo Rectification

Stereo rectification is an optional preprocessing step that transforms the images from both cameras so that corresponding
points appear on the same horizontal scanline. This simplifies stereo matching and can improve accuracy.

When enabled via the `stereo_rectify` option, the calibration module computes:

- `R1`, `R2`: Rectification rotation matrices for left and right cameras
- `P1`, `P2`: Rectified projection matrices for left and right cameras
- `Q`: Disparity-to-depth mapping matrix (4x4)

The rectification is computed using OpenCV's `stereoRectify` function during calibration parsing. Images are then
rectified during preprocessing using the `utils::rectify()` function.

### Usage

Enable stereo rectification via command line:
```bash
zenslam --stereo-rectify
```

Or in the YAML configuration file:
```yaml
slam:
  stereo_rectify: true
```

## Distortion & Undistortion

Images are undistorted before detection using camera calibration parameters. When stereo rectification is enabled,
the rectification process combines both distortion removal and image alignment in a single remapping step.
