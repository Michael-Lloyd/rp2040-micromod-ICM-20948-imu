#include <stdio.h>
#include "pico/stdlib.h"
#include "picoEKF.h"

using namespace picoEKF;

void simulate_imu_data(IMUMeasurement& meas, float time) {
    float omega = 0.5f;
    meas.gyro = Vector3(0.0f, 0.0f, omega);
    
    float angle = omega * time;
    meas.accel = Vector3(sin(angle) * 9.81f, cos(angle) * 9.81f, 0.0f);
    
    meas.mag = Vector3(cos(angle), -sin(angle), 0.0f);
    
    meas.dt = 0.01f;
}

int main() {
    stdio_init_all();
    
    sleep_ms(2000);
    printf("Pico EKF Example\n");
    printf("================\n\n");
    
    ExtendedKalmanFilter ekf;
    
    EKFConfig config;
    config.gyroNoise = 0.01f;
    config.gyroBiasNoise = 0.0001f;
    config.accelNoise = 0.1f;
    config.magNoise = 0.1f;
    config.gravityRef = Vector3(0, 0, 9.81f);
    config.magRef = Vector3(1, 0, 0);
    config.useMagnetometer = true;
    
    ekf.init(config);
    printf("EKF initialized\n\n");
    
    float time = 0.0f;
    const int num_iterations = 1000;
    
    printf("Running EKF with simulated IMU data...\n");
    printf("Time(s), Roll(deg), Pitch(deg), Yaw(deg), GyroBias_X, GyroBias_Y, GyroBias_Z\n");
    
    for (int i = 0; i < num_iterations; i++) {
        IMUMeasurement meas;
        simulate_imu_data(meas, time);
        
        ekf.update(meas);
        
        if (i % 100 == 0) {
            float roll, pitch, yaw;
            ekf.getEulerAngles(roll, pitch, yaw);
            
            Vector3 gyroBias = ekf.getGyroBias();
            
            roll *= 180.0f / M_PI;
            pitch *= 180.0f / M_PI;
            yaw *= 180.0f / M_PI;
            
            printf("%.2f, %.2f, %.2f, %.2f, %.6f, %.6f, %.6f\n", 
                   time, roll, pitch, yaw, 
                   gyroBias.x, gyroBias.y, gyroBias.z);
        }
        
        time += meas.dt;
        sleep_ms(10);
    }
    
    printf("\nEKF example completed!\n");
    
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}