# Pico EKF Library

A lightweight Extended Kalman Filter (EKF) library designed for attitude estimation on the RP2040 microcontroller. 
This library provides efficient implementations of EKF algorithms for fusing IMU sensor data 
(gyroscope, accelerometer, and optionally magnetometer) to estimate device orientation and gyroscope bias.

## Features

- **Multiple EKF Implementations**:
  - **Standard EKF**: Traditional Extended Kalman Filter implementation
  - **Iterative EKF**: Advanced implementation with iterative measurement updates for improved accuracy
  
- **6-DOF and 9-DOF Sensor Fusion**: Supports both accelerometer+gyroscope (6-DOF) and accelerometer+gyroscope+magnetometer (9-DOF) configurations

- **Gyroscope Bias Estimation**: Automatically estimates and compensates for gyroscope bias


## Library Structure

```
pico_ekf/
├── src/
│   ├── picoEKF.h              # Main header file
│   ├── picoEKF.cpp             # Factory implementation
│   ├── picoEKFTypes.h          # Common types and structures
│   ├── ExtendedKalmanFilter.h  # Abstract base class
│   ├── StandardEKF.h/cpp       # Standard EKF implementation
│   └── IterativeEKF.h/cpp      # Iterative EKF implementation
├── example/
│   ├── ekf_example.cpp         # Basic usage example
│   └── iterative_ekf_example.cpp # Iterative EKF example
└── CMakeLists.txt
```

## Installation

1. Add the library to your Pico project:
   ```cmake
   add_subdirectory(path/to/pico_ekf)
   ```

2. Link the library to your target:
   ```cmake
   target_link_libraries(your_target
       pico_stdlib
       pico_ekf
   )
   ```

## Quick Start

### Basic Usage

```cpp
#include "picoEKF.h"

using namespace picoEKF;

// Create an EKF instance
ExtendedKalmanFilter* ekf = createEKF(EKFType::Standard);

// Configure the filter
EKFConfig config;
config.gyroNoise = 0.01f;        // Gyroscope noise (rad/s)
config.gyroBiasNoise = 0.0001f;  // Gyroscope bias drift (rad/s)
config.accelNoise = 0.1f;        // Accelerometer noise (m/s²)
config.magNoise = 0.1f;          // Magnetometer noise (normalized)
config.gravityRef = Vector3(0, 0, 9.81f);  // Gravity reference vector
config.magRef = Vector3(1, 0, 0);          // Magnetic field reference
config.useMagnetometer = true;   // Enable 9-DOF fusion

// Initialize the filter
ekf->init(config);

// Update loop
while (true) {
    // Get sensor measurements
    IMUMeasurement meas;
    meas.gyro = Vector3(gx, gy, gz);      // rad/s
    meas.accel = Vector3(ax, ay, az);     // m/s²
    meas.mag = Vector3(mx, my, mz);       // normalized
    meas.dt = 0.01f;                      // time step (s)
    
    // Update the filter
    ekf->update(meas);
    
    // Get orientation estimate
    Quaternion q = ekf->getOrientation();
    
    // Get Euler angles
    float roll, pitch, yaw;
    ekf->getEulerAngles(roll, pitch, yaw);  // radians
    
    // Get gyroscope bias estimate
    Vector3 bias = ekf->getGyroBias();
}

// Clean up
delete ekf;
```

### Using Iterative EKF

The Iterative EKF provides improved accuracy through iterative measurement updates:

```cpp
// Create an Iterative EKF instance
ExtendedKalmanFilter* ekf = createEKF(EKFType::Iterative);

// Same configuration and usage as Standard EKF
// The Iterative EKF automatically performs multiple iterations
// during measurement updates for better convergence
```

## API Reference

### EKFType Enum
```cpp
enum class EKFType {
    Standard,   // Traditional EKF
    Iterative   // Iterative EKF with improved convergence
};
```

### EKFConfig Structure
```cpp
struct EKFConfig {
    float gyroNoise;         // Gyroscope measurement noise (rad/s)
    float gyroBiasNoise;     // Gyroscope bias drift noise (rad/s)
    float accelNoise;        // Accelerometer measurement noise (m/s²)
    float magNoise;          // Magnetometer measurement noise (normalized)
    Vector3 gravityRef;      // Gravity reference vector (m/s²)
    Vector3 magRef;          // Magnetic field reference (normalized)
    bool useMagnetometer;    // Enable magnetometer fusion
};
```

### IMUMeasurement Structure
```cpp
struct IMUMeasurement {
    Vector3 gyro;   // Gyroscope measurements (rad/s)
    Vector3 accel;  // Accelerometer measurements (m/s²)
    Vector3 mag;    // Magnetometer measurements (normalized)
    float dt;       // Time step since last measurement (s)
};
```

### ExtendedKalmanFilter Interface
```cpp
class ExtendedKalmanFilter {
    virtual void init(const EKFConfig& cfg) = 0;
    virtual void reset() = 0;
    virtual void update(const IMUMeasurement& measurement) = 0;
    virtual Quaternion getOrientation() const = 0;
    virtual Vector3 getGyroBias() const = 0;
    virtual void getEulerAngles(float& roll, float& pitch, float& yaw) const = 0;
    virtual bool isInitialized() const = 0;
};
```

## Tuning Guidelines

### Noise Parameters
- **gyroNoise**: Typical values 0.001-0.1 rad/s
- **gyroBiasNoise**: Typical values 0.00001-0.001 rad/s
- **accelNoise**: Typical values 0.01-1.0 m/s²
- **magNoise**: Typical values 0.01-0.5 (normalized)

## Examples

Two complete examples are provided in the `example/` directory:

1. **ekf_example.cpp**: Basic usage of the Standard EKF with simulated IMU data
2. **iterative_ekf_example.cpp**: Demonstrates the Iterative EKF and includes a performance comparison

To build the examples:
```bash
cd example
mkdir build && cd build
cmake ..
make
```

## Debug Mode

Enable debug output by defining `PICO_EKF_DEBUG`:
```cpp
#define PICO_EKF_DEBUG 1
```

This will enable detailed debug prints for troubleshooting filter behavior.
