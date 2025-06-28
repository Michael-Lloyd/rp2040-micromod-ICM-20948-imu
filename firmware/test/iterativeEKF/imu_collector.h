#ifndef IMU_COLLECTOR_H
#define IMU_COLLECTOR_H

#include "picoEKF.h"
#include "pico/mutex.h"
#include <stdint.h>
#include <cmath>

struct IMUData {
    picoEKF::Vector3 gyro;
    picoEKF::Vector3 accel;
    picoEKF::Vector3 mag;
    uint64_t timestamp;
    bool dataValid;
};

class IMUCollector {
private:
    enum MotionProfile {
        STATIONARY,
        SLOW_ROTATION,
        FAST_ROTATION,
        TILT_AND_ROTATE,
        RANDOM_MOTION
    };
    
    struct SimulationState {
        float roll, pitch, yaw;
        float rollRate, pitchRate, yawRate;
        picoEKF::Vector3 gyroBias;
        uint64_t startTime;
        MotionProfile currentProfile;
        uint64_t profileStartTime;
        uint32_t profileDuration;
    };
    
    SimulationState simState;
    uint64_t lastTime;
    
    mutex_t* dataMutex;
    IMUData* sharedData;
    
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;
    static constexpr float GYRO_NOISE_STD = 0.005f;
    static constexpr float ACCEL_NOISE_STD = 0.05f;
    static constexpr float MAG_NOISE_STD = 0.02f;
    
    float generateGaussianNoise(float stddev);
    void updateMotionProfile(uint64_t currentTime);
    void updateSimulationState(uint64_t currentTime);
    picoEKF::Quaternion eulerToQuaternion(float roll, float pitch, float yaw);
    picoEKF::IMUMeasurement generateMeasurement(uint64_t currentTimeUs);
    
public:
    IMUCollector(mutex_t* mutex, IMUData* data);
    
    void init();
    void collectData();
};

// Thread entry point for Core 0 IMU data collection (only used if running on separate core)
void core0_imu_thread();

#endif