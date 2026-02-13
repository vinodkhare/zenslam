# IMU Pre-integration Implementation

## Overview

The IMU pre-integration system uses the [ugpm library](https://github.com/UTS-RI/ugpm) to perform accurate IMU measurement integration. The implementation is encapsulated in the `preint` class which handles the complexity of UGPM/LPM preintegration methods.

## Architecture

### Core Components

1. **`zenslam::preint` class** (`zenslam_core/include/zenslam/preint.h`)
   - Encapsulates IMU preintegration logic
   - Supports both UGPM (Gaussian Process) and LPM (Linear) methods
   - Manages overlap data for optimal UGPM performance
   - Provides graceful degradation when ugpm is unavailable

2. **Integration in `utils::process()`** (`zenslam_core/source/utils_slam.cpp`)
   - Creates preint instance per frame
   - Configures optimal parameters (8x overlap, 50Hz state frequency)
   - Stores result in `frame::processed.preint`

3. **CMake Configuration** (`zenslam_core/CMakeLists.txt`)
   - Fetches ugpm from GitHub using FetchContent
   - Creates INTERFACE target for header-only ugpm library
   - Links Ceres (required dependency for ugpm headers)

## Usage

### Basic Integration

```cpp
// Create preintegrator with calibration
preint imu_preint(calibration.imu, preint::method::ugpm);

// Configure (optional - these are defaults)
imu_preint.set_overlap_factor(8);     // 8x state period overlap
imu_preint.set_state_frequency(50.0); // 50 Hz state frequency
imu_preint.set_correlate(true);       // Enable correlation

// Integrate measurements
auto result = imu_preint.integrate(imu_measurements, start_time, end_time);

if (result.has_value())
{
    // Use preintegrated measurement
    ugpm::PreintMeas preint_meas = result.value();
    // Access: preint_meas.delta_R, delta_v, delta_p, cov
}
```

### With Overlap (Recommended for UGPM)

```cpp
// For optimal UGPM accuracy, use overlap between windows
static preint imu_preint(calibration.imu, preint::method::ugpm);

// Maintains overlap state across calls
auto result = imu_preint.integrate_with_overlap(imu_measurements, start_time, end_time);
```

### Pose Prediction

```cpp
// Given current state
cv::Affine3d pose_0 = current_pose;
cv::Vec3d velocity_0(vx, vy, vz);  // World frame velocity

// Integrate IMU measurements
auto preint_result = imu_preint.integrate(imu_data, t0, t1);

if (preint_result.has_value())
{
    // Predict next pose
    cv::Affine3d predicted_pose = preint::predict_pose(
        pose_0,
        velocity_0,
        preint_result.value()
    );

    // Predict next velocity
    cv::Vec3d predicted_velocity = preint::predict_velocity(
        pose_0,
        velocity_0,
        preint_result.value()
    );

    // Use predictions as initial guess for visual odometry
    // or standalone dead reckoning
}
```

### Convenience Utility

```cpp
// Higher-level API using frame data directly
cv::Affine3d predicted_pose = utils::predict_pose_from_imu(
    current_pose,
    current_velocity,
    processed_frame  // Contains preintegrated IMU data
);
```

## Configuration

### Methods

- **UGPM** (Gaussian Process-based)
  - More accurate, especially with irregular sampling
  - Slower computation
  - Requires overlap between windows (8×state_period recommended)
  - Recommended for high-accuracy applications

- **LPM** (Linear Pre-integration)
  - Faster computation
  - Standard linear integration
  - No overlap required
  - Suitable for real-time applications

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `overlap_factor` | 8 | Multiplier for state period to determine overlap |
| `state_frequency` | 50.0 Hz | Frequency of the state in the Gaussian process |
| `correlate` | true | Enable correlation in covariance estimation |

### Noise Model

IMU noise is configured from `imu_calibration`:
```cpp
acc_var = accelerometer_noise_density²
gyr_var = gyroscope_noise_density²
```

## Output

The preintegrated measurement (`ugpm::PreintMeas`) contains:

- `delta_R`: Rotation change (SO(3) matrix)
- `delta_v`: Velocity change (3D vector)
- `delta_p`: Position change (3D vector)
- `dt`: Time interval
- `dt_sq_half`: dt²/2 for convenience
- `cov`: 9×9 covariance matrix for [delta_R, delta_v, delta_p]

## Overlap Mechanism

For UGPM to perform optimally, consecutive integration windows should overlap:

```
Window 1: [t0 -------- overlap -------- t1]
Window 2:              [t1 -------- overlap -------- t2]
                                     [t2 -------- overlap -------- t3]
```

The `integrate_with_overlap()` method:
1. Stores measurements from previous integration
2. Computes overlap start time: `start_time - (overlap_factor / state_freq)`
3. Combines previous measurements (after overlap start) with current measurements
4. Performs preintegration over the combined data

## Fallback Behavior

When ugpm is not available (ZENSLAM_HAS_UGPM=0):
- `integrate()` returns `std::nullopt`
- `integrate_with_overlap()` returns `std::nullopt`
- `identity()` returns a PreintMeas with identity rotation and zero velocity/position
- Warning logged: "ugpm library not available, IMU preintegration disabled"

## Dependencies

- **ugpm**: Header-only IMU preintegration library
- **Ceres**: Required by ugpm headers (even when using LPM)
- **Eigen**: Matrix library (vendored within ugpm)

## Pose Prediction Mathematics

The IMU pre-integration provides delta measurements in the body frame, which are propagated to predict the next state:

### Rotation Prediction
```
R_1 = R_0 * delta_R
```
Where:
- `R_0`: Rotation at time t0 (world frame)
- `delta_R`: Preintegrated rotation change (body frame)
- `R_1`: Predicted rotation at time t1

### Velocity Prediction
```
v_1 = v_0 + R_0 * delta_v + g * dt
```
Where:
- `v_0`: Velocity at t0 (world frame)
- `delta_v`: Preintegrated velocity change (body frame)
- `g`: Gravity vector (world frame, typically [0, 0, -9.81] m/s²)
- `dt`: Time interval

### Position Prediction
```
p_1 = p_0 + v_0 * dt + R_0 * delta_p + 0.5 * g * dt²
```
Where:
- `p_0`: Position at t0 (world frame)
- `delta_p`: Preintegrated position change (body frame)

### Key Insight
The body-frame measurements (`delta_R`, `delta_v`, `delta_p`) are rotated into the world frame using `R_0`, then gravity is added since IMU measures specific force (acceleration minus gravity).

## Future Enhancements

1. **Parameter Exposure**: Add `options::slam` fields for overlap_factor, state_freq, method selection
2. **State Management**: Consider thread-local or singleton preint instances for overlap continuity
3. **Performance Tuning**: Benchmark UGPM vs LPM on target hardware
4. **Unit Tests**: Add comprehensive tests for preint class (see `zenslam_tests/`)
5. **Visual-Inertial Fusion**: ✅ Pose prediction implemented - integrate with tracking pipeline
6. **Bias Estimation**: Online estimation and correction of IMU biases
7. **Keyframe-based Integration**: Accumulate preintegration over keyframe intervals
