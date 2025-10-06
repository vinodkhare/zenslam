# Roadmap

## Short Term
- Solidify triangulation & add cheirality + reprojection pruning
- Implement pose estimation (Essential matrix decomposition or PnP with 3D-2D correspondences)
- Introduce a lightweight map structure (landmarks + observations)
- Persistent track IDs (unifying detection + KLT tracking)

## Medium Term
- Keyframe selection heuristics
- Local bundle adjustment
- Loop closure detection (BoW or learned descriptors)
- Optional GPU acceleration for feature pipeline

## Long Term / Stretch
- Multi-camera / IMU fusion
- Online calibration refinement
- Dense or semi-dense depth integration
- SLAM evaluation harness with dataset metrics (ATE, RPE)

## Guiding Principles
- Start minimal; measure before optimizing.
- Keep modules orthogonal (IO, detection, geometry, optimization separate).
- Embrace replaceable components (detectors, matchers, solvers).
