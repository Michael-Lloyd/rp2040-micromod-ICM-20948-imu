#ifndef EKF_PROCESSOR_H
#define EKF_PROCESSOR_H

#include "picoEKF.h"
#include "imu_collector.h"
#include "pico/mutex.h"
#include <stdint.h>

class EKFProcessor {
private:
    picoEKF::StandardEKF ekf;
    mutex_t* dataMutex;
    IMUData* sharedData;
    
    uint64_t lastTimestamp;
    bool firstUpdate;
    
    // Reference vectors (to be calibrated)
    picoEKF::Vector3 gravityRef;
    picoEKF::Vector3 magRef;
    
public:
    EKFProcessor(mutex_t* mutex, IMUData* data);
    
    void init();
    void processUpdate();
    
    // Get current state estimates
    void getOrientation(float& roll, float& pitch, float& yaw);
    picoEKF::Quaternion getQuaternion();
    picoEKF::Vector3 getGyroBias();
    
    // Configuration
    void setGravityReference(const picoEKF::Vector3& ref);
    void setMagneticReference(const picoEKF::Vector3& ref);
};

// Thread entry point for Core 1 EKF processing
void core1_ekf_thread();

#endif