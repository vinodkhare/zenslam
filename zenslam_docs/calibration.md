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

## Distortion & Undistortion
Currently an undistortion helper prepares images before detection. Future improvements may include rectification or model switching.
