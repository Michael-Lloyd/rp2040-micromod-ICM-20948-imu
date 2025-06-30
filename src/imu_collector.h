#ifndef IMU_COLLECTOR_H
#define IMU_COLLECTOR_H

#include "ICM_20948.h"
#include "picoEKF.h"
#include "pico/mutex.h"
#include <stdint.h>

// Shared data structure for thread-safe IMU data passing
struct IMUData {
    picoEKF::Vector3 gyro;      // rad/s
    picoEKF::Vector3 accel;     // m/s^2
    picoEKF::Vector3 mag;       // normalized
    uint64_t timestamp;         // microseconds
    bool dataValid;
};

class IMUCollector {
private:
    ICM_20948_I2C* icm;
    mutex_t* dataMutex;
    IMUData* sharedData;
    
    // Conversion factors
    static constexpr float DEG_TO_RAD = 0.017453292519943295f;
    static constexpr float G_TO_MS2 = 9.80665f;
    
public:
    IMUCollector(ICM_20948_I2C* sensor, mutex_t* mutex, IMUData* data);
    
    void init();
    void collectData();
    
    // Convert raw IMU readings to SI units for EKF
    picoEKF::Vector3 convertGyro(float x, float y, float z);
    picoEKF::Vector3 convertAccel(float x, float y, float z);
    picoEKF::Vector3 normalizeMag(float x, float y, float z);
};

// Thread entry point
void imu_collector_thread();

#endif