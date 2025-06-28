#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/multicore.h>
#include <pico/mutex.h>
#include "ekf_processor.h"
#include "imu_collector.h"

static mutex_t dataMutex;
static IMUData sharedData = {
    .gyro = {0, 0, 0},
    .accel = {0, 0, 0},
    .mag = {0, 0, 0},
    .timestamp = 0,
    .dataValid = false
};

static IMUCollector* g_imuSim = nullptr;
static EKFProcessor* g_ekfProc = nullptr;

void core1_entry() {
    core1_ekf_thread();
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("IterativeEKF Test with Synthetic IMU Data\n");
    printf("=========================================\n");
    printf("Using multicore architecture:\n");
    printf("- Core 0: Synthetic IMU data generation and display\n");
    printf("- Core 1: IterativeEKF processing\n\n");
    
    mutex_init(&dataMutex);
    
    // Initialize synthetic IMU data generator
    g_imuSim = new IMUCollector(&dataMutex, &sharedData);
    g_imuSim->init();
    
    // Initialize EKF processor (uses IterativeEKF implementation)
    g_ekfProc = new EKFProcessor(&dataMutex, &sharedData);
    g_ekfProc->init();
    
    printf("Starting EKF processor on Core 1...\n");
    multicore_launch_core1(core1_entry);
    
    sleep_ms(100);
    
    printf("Starting IMU collector on Core 0...\n");
    printf("System running. Displaying orientation data:\n\n");
    
    uint64_t lastPrintTime = time_us_64();
    uint64_t lastCollectTime = time_us_64();
    const uint64_t printPeriodUs = 100000;
    const uint64_t collectPeriodUs = 10000;
    
    while (true) {
        uint64_t currentTime = time_us_64();
        
        if (currentTime - lastCollectTime >= collectPeriodUs) {
            g_imuSim->collectData();
            lastCollectTime = currentTime;
        }
        
        if (currentTime - lastPrintTime >= printPeriodUs) {
            float roll, pitch, yaw;
            g_ekfProc->getOrientation(roll, pitch, yaw);
            
            picoEKF::Vector3 gyroBias = g_ekfProc->getGyroBias();
            
            float elapsedSec = currentTime / 1000000.0f;
            
            printf("[%.2f s] Roll: %6.1f°, Pitch: %6.1f°, Yaw: %6.1f° | ",
                   elapsedSec, roll * 180.0f / M_PI, pitch * 180.0f / M_PI, yaw * 180.0f / M_PI);
            printf("Bias: [%.4f, %.4f, %.4f]\n",
                   gyroBias.x, gyroBias.y, gyroBias.z);
            
            lastPrintTime = currentTime;
        }
        
        sleep_ms(10);
    }
    
    return 0;
}