#include "imu_collector.h"
#include "pico/stdlib.h"
#include <cmath>

// Global instances for thread access
static ICM_20948_I2C* g_icm = nullptr;
static mutex_t* g_dataMutex = nullptr;
static IMUData* g_sharedData = nullptr;

IMUCollector::IMUCollector(ICM_20948_I2C* sensor, mutex_t* mutex, IMUData* data)
    : icm(sensor), dataMutex(mutex), sharedData(data) {
    // Set global pointers for thread access
    g_icm = sensor;
    g_dataMutex = mutex;
    g_sharedData = data;
}

void IMUCollector::init() {
    // Configure sample rates
    ICM_20948_smplrt_t sampleRate;
    sampleRate.g = 19;  // Gyro: 1125Hz / (1 + 19) = 56.25Hz
    sampleRate.a = 19;  // Accel: 1125Hz / (1 + 19) = 56.25Hz
    icm->setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, sampleRate);
    
    // Initialize shared data
    sharedData->dataValid = false;
    sharedData->timestamp = 0;
}

void IMUCollector::collectData() {
    if (icm->dataReady()) {
        // Read all sensor data
        icm->getAGMT();
        
        // Get current timestamp
        uint64_t timestamp = time_us_64();
        
        // Convert to SI units
        picoEKF::Vector3 gyroData = convertGyro(icm->gyrX(), icm->gyrY(), icm->gyrZ());
        picoEKF::Vector3 accelData = convertAccel(icm->accX(), icm->accY(), icm->accZ());
        picoEKF::Vector3 magData = normalizeMag(icm->magX(), icm->magY(), icm->magZ());
        
        // Lock mutex and update shared data
        mutex_enter_blocking(dataMutex);
        sharedData->gyro = gyroData;
        sharedData->accel = accelData;
        sharedData->mag = magData;
        sharedData->timestamp = timestamp;
        sharedData->dataValid = true;
        mutex_exit(dataMutex);
    }
}

picoEKF::Vector3 IMUCollector::convertGyro(float x, float y, float z) {
    // Convert from degrees/sec to rad/sec
    return picoEKF::Vector3(x * DEG_TO_RAD, y * DEG_TO_RAD, z * DEG_TO_RAD);
}

picoEKF::Vector3 IMUCollector::convertAccel(float x, float y, float z) {
    // Convert from mg to m/s^2
    return picoEKF::Vector3(
        x * 0.001f * G_TO_MS2,
        y * 0.001f * G_TO_MS2,
        z * 0.001f * G_TO_MS2
    );
}

picoEKF::Vector3 IMUCollector::normalizeMag(float x, float y, float z) {
    // Normalize magnetometer readings
    picoEKF::Vector3 mag(x, y, z);
    float norm = mag.norm();
    if (norm > 0) {
        return mag * (1.0f / norm);
    }
    return mag;
}

void imu_collector_thread() {
    if (!g_icm || !g_dataMutex || !g_sharedData) {
        return;
    }
    
    IMUCollector collector(g_icm, g_dataMutex, g_sharedData);
    collector.init();
    
    // Main collection loop
    while (true) {
        collector.collectData();
        
        // Small delay to prevent I2C bus saturation
        sleep_us(500);
    }
}