# IMU Calibration

ZenSLAM provides support for reading IMU (Inertial Measurement Unit) calibration parameters from YAML files. These parameters are essential for sensor fusion, visual-inertial odometry, and state estimation algorithms.

## Overview

The `imu_calibration` class provides functionality to parse and store IMU noise and bias parameters that are typically obtained from Allan variance analysis. These parameters characterize the sensor noise properties and are used in Kalman filters and other estimation algorithms.

## Parsing

`imu_calibration::parse(path)` loads an IMU configuration YAML file and populates an `imu_calibration` object.

### Example Usage

```cpp
#include <zenslam/imu_calibration.h>

// Parse IMU calibration from file
auto imu_calib = zenslam::imu_calibration::parse("/path/to/imu_config.yaml");

// Print parameters to log
imu_calib.print();

// Access individual parameters
double gyro_noise = imu_calib.gyroscope_noise_density;
double accel_noise = imu_calib.accelerometer_noise_density;
```

## Configuration File Format

The IMU configuration file is a YAML file with the following structure:

```yaml
rostopic: /imu0
update_rate: 200.0  # Hz

# Accelerometer parameters
accelerometer_noise_density: 0.0028     # m/s^1.5
accelerometer_random_walk:   0.00086    # m/s^2.5

# Gyroscope parameters
gyroscope_noise_density:     0.00016    # rad/s^0.5
gyroscope_random_walk:       0.000022   # rad/s^1.5
```

### Parameter Descriptions

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `rostopic` | string | - | ROS topic name for the IMU (optional) |
| `update_rate` | double | Hz | IMU sampling rate (optional, default: 200.0) |
| `accelerometer_noise_density` | double | m/s^1.5 | Accelerometer white noise density |
| `accelerometer_random_walk` | double | m/s^2.5 | Accelerometer bias random walk |
| `gyroscope_noise_density` | double | rad/s^0.5 | Gyroscope white noise density |
| `gyroscope_random_walk` | double | rad/s^1.5 | Gyroscope bias random walk |

## Integration with Options System

The IMU calibration file can be specified through:

### 1. Command Line Arguments

```bash
./zenslam_app --imu-calibration-file=/path/to/imu_config.yaml
```

### 2. YAML Options File

Add to your main options file (e.g., `options.yaml`):

```yaml
folder:
  imu_calibration_file: imu_config.yaml
```

### 3. Programmatic Access

```cpp
#include <zenslam/options.h>
#include <zenslam/imu_calibration.h>

// Parse options (includes imu_calibration_file path)
auto opts = zenslam::options::parse(argc, argv);

// Load IMU calibration
auto imu_calib = zenslam::imu_calibration::parse(opts.folder.imu_calibration_file);
```

## Stored Fields

```cpp
class imu_calibration
{
    std::string rostopic;                         // ROS topic name
    double update_rate;                           // Hz
    double accelerometer_noise_density;           // m/s^1.5
    double accelerometer_random_walk;             // m/s^2.5
    double gyroscope_noise_density;               // rad/s^0.5
    double gyroscope_random_walk;                 // rad/s^1.5
};
```

## Obtaining IMU Parameters

IMU noise parameters are typically obtained through:

1. **Allan Variance Analysis**: Record static IMU data for several hours and analyze the Allan deviation plot to extract noise and bias parameters.

2. **Manufacturer Specifications**: Some IMU manufacturers provide noise specifications in datasheets.

3. **Kalibr Calibration**: The [Kalibr](https://github.com/ethz-asl/kalibr) toolbox can estimate IMU noise parameters from static recordings.

## Example: TUM-VI Dataset

For the TUM-VI dataset, a typical IMU configuration file looks like:

```yaml
rostopic: /imu0
update_rate: 200.0

# Allan variance analysis results
accelerometer_noise_density: 0.0014
accelerometer_random_walk:   0.000086
gyroscope_noise_density:     0.000080
gyroscope_random_walk:       0.0000022

# Note: Values may be inflated (e.g., multiplied by 2-10x) to account
# for unmodelled effects in practical applications
```

## Error Handling

The parser will throw a `std::runtime_error` if:
- The specified file does not exist
- The file cannot be parsed as valid YAML

Optional fields (like `rostopic`) will use default values if not present in the file.

## Future Extensions

Potential future enhancements include:

- Support for additional IMU parameters (e.g., temperature coefficients)
- Integration with visual-inertial SLAM pipeline
- Online IMU bias estimation and calibration refinement
- Support for multiple IMU configurations (e.g., primary and backup sensors)

## References

- [Allan Variance: Noise Analysis for Gyroscopes](http://www.nxp.com/docs/en/application-note/AN5087.pdf)
- [Kalibr: Camera-IMU Calibration](https://github.com/ethz-asl/kalibr)
- [TUM Visual-Inertial Dataset](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
