#include "imu_collector.h"
#include <stdlib.h>
#include <pico/time.h>

using namespace picoEKF;

static IMUCollector* g_collector = nullptr;
static mutex_t* g_dataMutex = nullptr;
static IMUData* g_sharedData = nullptr;

IMUCollector::IMUCollector(mutex_t* mutex, IMUData* data)
    : dataMutex(mutex), sharedData(data) {
    simState.roll = 0;
    simState.pitch = 0;
    simState.yaw = 0;
    simState.rollRate = 0;
    simState.pitchRate = 0;
    simState.yawRate = 0;
    simState.gyroBias = Vector3(0.01f, -0.005f, 0.002f);
    simState.currentProfile = STATIONARY;
    simState.profileDuration = 5000000;
    lastTime = 0;
}

void IMUCollector::init() {
    simState.startTime = time_us_64();
    simState.profileStartTime = simState.startTime;
    
    srand(12345);
    
    g_collector = this;
    g_dataMutex = dataMutex;
    g_sharedData = sharedData;
}

float IMUCollector::generateGaussianNoise(float stddev) {
    static bool hasSpare = false;
    static float spare;
    
    if (hasSpare) {
        hasSpare = false;
        return spare * stddev;
    }
    
    hasSpare = true;
    float u, v, s;
    do {
        u = (rand() / ((float)RAND_MAX)) * 2.0f - 1.0f;
        v = (rand() / ((float)RAND_MAX)) * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);
    
    s = sqrtf(-2.0f * logf(s) / s);
    spare = v * s;
    return u * s * stddev;
}

void IMUCollector::updateMotionProfile(uint64_t currentTime) {
    if (currentTime - simState.profileStartTime > simState.profileDuration) {
        simState.profileStartTime = currentTime;
        
        simState.currentProfile = (MotionProfile)((int)simState.currentProfile + 1);
        if (simState.currentProfile > RANDOM_MOTION) {
            simState.currentProfile = STATIONARY;
        }
        
        switch (simState.currentProfile) {
            case STATIONARY:
                simState.rollRate = 0;
                simState.pitchRate = 0;
                simState.yawRate = 0;
                simState.profileDuration = 3000000;
                break;
                
            case SLOW_ROTATION:
                simState.rollRate = 10.0f * DEG_TO_RAD;
                simState.pitchRate = 0;
                simState.yawRate = 20.0f * DEG_TO_RAD;
                simState.profileDuration = 5000000;
                break;
                
            case FAST_ROTATION:
                simState.rollRate = 0;
                simState.pitchRate = 45.0f * DEG_TO_RAD;
                simState.yawRate = 90.0f * DEG_TO_RAD;
                simState.profileDuration = 4000000;
                break;
                
            case TILT_AND_ROTATE:
                simState.rollRate = 30.0f * DEG_TO_RAD * sinf(currentTime * 0.0000005f);
                simState.pitchRate = 20.0f * DEG_TO_RAD * cosf(currentTime * 0.0000003f);
                simState.yawRate = 50.0f * DEG_TO_RAD;
                simState.profileDuration = 6000000;
                break;
                
            case RANDOM_MOTION:
                simState.rollRate = (rand() / (float)RAND_MAX - 0.5f) * 100.0f * DEG_TO_RAD;
                simState.pitchRate = (rand() / (float)RAND_MAX - 0.5f) * 100.0f * DEG_TO_RAD;
                simState.yawRate = (rand() / (float)RAND_MAX - 0.5f) * 100.0f * DEG_TO_RAD;
                simState.profileDuration = 2000000;
                break;
        }
    }
}

void IMUCollector::updateSimulationState(uint64_t currentTime) {
    if (lastTime == 0) {
        lastTime = currentTime;
        simState.startTime = currentTime;
        return;
    }
    
    float dt = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;
    
    updateMotionProfile(currentTime);
    
    simState.roll += simState.rollRate * dt;
    simState.pitch += simState.pitchRate * dt;
    simState.yaw += simState.yawRate * dt;
    
    while (simState.roll > M_PI) simState.roll -= 2 * M_PI;
    while (simState.roll < -M_PI) simState.roll += 2 * M_PI;
    while (simState.pitch > M_PI) simState.pitch -= 2 * M_PI;
    while (simState.pitch < -M_PI) simState.pitch += 2 * M_PI;
    while (simState.yaw > M_PI) simState.yaw -= 2 * M_PI;
    while (simState.yaw < -M_PI) simState.yaw += 2 * M_PI;
}

Quaternion IMUCollector::eulerToQuaternion(float roll, float pitch, float yaw) {
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

IMUMeasurement IMUCollector::generateMeasurement(uint64_t currentTimeUs) {
    updateSimulationState(currentTimeUs);
    
    IMUMeasurement meas;
    
    meas.gyro.x = simState.rollRate + simState.gyroBias.x + generateGaussianNoise(GYRO_NOISE_STD);
    meas.gyro.y = simState.pitchRate + simState.gyroBias.y + generateGaussianNoise(GYRO_NOISE_STD);
    meas.gyro.z = simState.yawRate + simState.gyroBias.z + generateGaussianNoise(GYRO_NOISE_STD);
    
    Quaternion q = eulerToQuaternion(simState.roll, simState.pitch, simState.yaw);
    Vector3 gravity(0, 0, -9.81f);
    Vector3 magField(0.22f, 0, -0.97f);
    
    Vector3 bodyGravity = q.conjugate().rotate(gravity);
    Vector3 bodyMag = q.conjugate().rotate(magField);
    
    meas.accel.x = bodyGravity.x + generateGaussianNoise(ACCEL_NOISE_STD);
    meas.accel.y = bodyGravity.y + generateGaussianNoise(ACCEL_NOISE_STD);
    meas.accel.z = bodyGravity.z + generateGaussianNoise(ACCEL_NOISE_STD);
    
    meas.mag = bodyMag.normalized();
    meas.mag.x += generateGaussianNoise(MAG_NOISE_STD);
    meas.mag.y += generateGaussianNoise(MAG_NOISE_STD);
    meas.mag.z += generateGaussianNoise(MAG_NOISE_STD);
    meas.mag = meas.mag.normalized();
    
    meas.dt = (currentTimeUs - simState.startTime) / 1000000.0f;
    if (meas.dt > 1.0f) meas.dt = 0.01f;
    
    return meas;
}

void IMUCollector::collectData() {
    uint64_t currentTime = time_us_64();
    IMUMeasurement meas = generateMeasurement(currentTime);
    
    mutex_enter_blocking(dataMutex);
    
    sharedData->gyro = meas.gyro;
    sharedData->accel = meas.accel;
    sharedData->mag = meas.mag;
    sharedData->timestamp = currentTime;
    sharedData->dataValid = true;
    
    mutex_exit(dataMutex);
}

void core0_imu_thread() {
    if (!g_collector || !g_dataMutex || !g_sharedData) {
        return;
    }
    
    while (true) {
        g_collector->collectData();
        sleep_ms(10);
    }
}