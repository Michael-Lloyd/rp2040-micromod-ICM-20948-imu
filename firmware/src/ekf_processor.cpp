#include "ekf_processor.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Global instances for thread access
static mutex_t* g_dataMutex = nullptr;
static IMUData* g_sharedData = nullptr;
static EKFProcessor* g_processor = nullptr;

EKFProcessor::EKFProcessor(mutex_t* mutex, IMUData* data)
    : dataMutex(mutex), sharedData(data), lastTimestamp(0), firstUpdate(true) {
    // Set global pointers for thread access
    g_dataMutex = mutex;
    g_sharedData = data;
    g_processor = this;
    
    // Default reference vectors
    gravityRef = picoEKF::Vector3(0, 0, 9.81f);
    magRef = picoEKF::Vector3(1, 0, 0);  // Will be calibrated from first reading
}

void EKFProcessor::init() {
    picoEKF::EKFConfig config;
    
    // Configure noise parameters
    config.gyroNoise = 0.01f;
    config.gyroBiasNoise = 0.0001f;
    config.accelNoise = 0.1f;
    config.magNoise = 0.1f;
    
    // Set reference vectors
    config.gravityRef = gravityRef;
    config.magRef = magRef;
    config.useMagnetometer = true;
    
    // Initialize the EKF
    ekf.init(config);
}

void EKFProcessor::processUpdate() {
    IMUData localData;
    bool hasData = false;
    
    // Get data with minimal mutex lock time
    mutex_enter_blocking(dataMutex);
    if (sharedData->dataValid) {
        localData = *sharedData;
        hasData = true;
    }
    mutex_exit(dataMutex);
    
    if (!hasData) {
        return;
    }
    
    // Calibrate magnetic reference on first reading
    if (firstUpdate && localData.mag.norm() > 0) {
        magRef = localData.mag;
        picoEKF::EKFConfig config;
        config.gyroNoise = 0.01f;
        config.gyroBiasNoise = 0.0001f;
        config.accelNoise = 0.1f;
        config.magNoise = 0.1f;
        config.gravityRef = gravityRef;
        config.magRef = magRef;
        config.useMagnetometer = true;
        ekf.init(config);
    }
    
    // Calculate time delta
    float dt = 0.01f;  // Default 10ms
    if (!firstUpdate) {
        dt = (localData.timestamp - lastTimestamp) * 1e-6f;  // Convert us to s
        
        // Clamp dt to reasonable values
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f;
    }
    
    // Create measurement for EKF
    picoEKF::IMUMeasurement measurement;
    measurement.gyro = localData.gyro;
    measurement.accel = localData.accel;
    measurement.mag = localData.mag;
    measurement.dt = dt;
    
    // Update EKF
    ekf.update(measurement);
    
    lastTimestamp = localData.timestamp;
    firstUpdate = false;
}

void EKFProcessor::getOrientation(float& roll, float& pitch, float& yaw) {
    ekf.getEulerAngles(roll, pitch, yaw);
}

picoEKF::Quaternion EKFProcessor::getQuaternion() {
    return ekf.getOrientation();
}

picoEKF::Vector3 EKFProcessor::getGyroBias() {
    return ekf.getGyroBias();
}

void EKFProcessor::setGravityReference(const picoEKF::Vector3& ref) {
    gravityRef = ref;
}

void EKFProcessor::setMagneticReference(const picoEKF::Vector3& ref) {
    magRef = ref;
}

void ekf_processor_thread() {
    if (!g_dataMutex || !g_sharedData || !g_processor) {
        return;
    }
    
    g_processor->init();
    
    uint32_t updateCount = 0;
    absolute_time_t lastPrintTime = get_absolute_time();
    
    // Main processing loop
    while (true) {
        g_processor->processUpdate();
        
        // Print status every 200ms
        if (absolute_time_diff_us(lastPrintTime, get_absolute_time()) >= 200000) {
            lastPrintTime = get_absolute_time();
            
            float roll, pitch, yaw;
            g_processor->getOrientation(roll, pitch, yaw);
            picoEKF::Vector3 bias = g_processor->getGyroBias();
            
            printf("EKF Update #%lu\n", updateCount);
            printf(" Orientation (deg): Roll=%7.2f  Pitch=%7.2f  Yaw=%7.2f\n", 
                   roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f);
            printf(" Gyro Bias (dps):  X=%7.3f  Y=%7.3f  Z=%7.3f\n\n",
                   bias.x * 57.2958f, bias.y * 57.2958f, bias.z * 57.2958f);
        }
        
        updateCount++;
        
        // Run at approximately 100Hz
        sleep_ms(10);
    }
}