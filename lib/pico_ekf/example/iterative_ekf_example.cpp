#include <stdio.h>
#include "pico/stdlib.h"
#include "picoEKF.h"

using namespace picoEKF;

// Simulate more challenging IMU data with noise and bias
void simulate_challenging_imu_data(IMUMeasurement& meas, float time, 
                                  const Vector3& true_bias,
                                  float noise_scale = 1.0f) {
    // True angular velocity with varying profile
    float omega_z = 0.5f + 0.2f * sin(time * 0.5f);  // Varying rotation rate
    float omega_x = 0.1f * sin(time * 0.3f);         // Small pitch oscillation
    float omega_y = 0.05f * cos(time * 0.4f);        // Small roll oscillation
    
    // Add bias and noise to gyroscope measurements
    meas.gyro = Vector3(
        omega_x + true_bias.x + noise_scale * 0.01f * (rand() / (float)RAND_MAX - 0.5f),
        omega_y + true_bias.y + noise_scale * 0.01f * (rand() / (float)RAND_MAX - 0.5f),
        omega_z + true_bias.z + noise_scale * 0.01f * (rand() / (float)RAND_MAX - 0.5f)
    );
    
    // Simulate accelerometer readings with tilt and noise
    float angle = omega_z * time;
    float tilt = 0.1f * sin(time * 0.2f);  // Small tilt variation
    
    meas.accel = Vector3(
        sin(angle) * 9.81f * cos(tilt) + noise_scale * 0.1f * (rand() / (float)RAND_MAX - 0.5f),
        cos(angle) * 9.81f * cos(tilt) + noise_scale * 0.1f * (rand() / (float)RAND_MAX - 0.5f),
        9.81f * sin(tilt) + noise_scale * 0.1f * (rand() / (float)RAND_MAX - 0.5f)
    );
    
    // Simulate magnetometer readings with disturbance
    float mag_disturbance = 0.2f * sin(time * 0.1f);
    meas.mag = Vector3(
        cos(angle) + mag_disturbance + noise_scale * 0.05f * (rand() / (float)RAND_MAX - 0.5f),
        -sin(angle) + noise_scale * 0.05f * (rand() / (float)RAND_MAX - 0.5f),
        0.5f + noise_scale * 0.05f * (rand() / (float)RAND_MAX - 0.5f)
    );
    
    meas.dt = 0.01f;  // 100 Hz update rate
}

// Compare two EKF implementations
void compare_ekf_performance() {
    printf("\nComparing Standard EKF vs Iterative EKF Performance\n");
    printf("==================================================\n\n");
    
    // Create both EKF types
    ExtendedKalmanFilter* standard_ekf = createEKF(EKFType::Standard);
    ExtendedKalmanFilter* iterative_ekf = createEKF(EKFType::Iterative);
    
    // Configure both with same parameters
    EKFConfig config;
    config.gyroNoise = 0.01f;
    config.gyroBiasNoise = 0.0001f;
    config.accelNoise = 0.1f;
    config.magNoise = 0.1f;
    config.gravityRef = Vector3(0, 0, 9.81f);
    config.magRef = Vector3(1, 0, 0).normalized();
    config.useMagnetometer = true;
    
    standard_ekf->init(config);
    iterative_ekf->init(config);
    
    // True gyroscope bias (to be estimated)
    Vector3 true_bias(0.02f, -0.01f, 0.015f);
    
    float time = 0.0f;
    const int num_iterations = 2000;
    
    printf("Time(s), Standard_Yaw(deg), Iterative_Yaw(deg), ");
    printf("Std_BiasZ, Iter_BiasZ, True_BiasZ\n");
    
    for (int i = 0; i < num_iterations; i++) {
        IMUMeasurement meas;
        simulate_challenging_imu_data(meas, time, true_bias, 1.0f);
        
        // Update both filters
        standard_ekf->update(meas);
        iterative_ekf->update(meas);
        
        // Print comparison every 200 iterations
        if (i % 200 == 0) {
            float std_roll, std_pitch, std_yaw;
            float iter_roll, iter_pitch, iter_yaw;
            
            standard_ekf->getEulerAngles(std_roll, std_pitch, std_yaw);
            iterative_ekf->getEulerAngles(iter_roll, iter_pitch, iter_yaw);
            
            Vector3 std_bias = standard_ekf->getGyroBias();
            Vector3 iter_bias = iterative_ekf->getGyroBias();
            
            printf("%.2f, %.2f, %.2f, %.4f, %.4f, %.4f\n", 
                   time, 
                   std_yaw * 180.0f / M_PI, 
                   iter_yaw * 180.0f / M_PI,
                   std_bias.z, iter_bias.z, true_bias.z);
        }
        
        time += meas.dt;
    }
    
    // Final comparison
    printf("\nFinal Results:\n");
    printf("--------------\n");
    
    Vector3 std_final_bias = standard_ekf->getGyroBias();
    Vector3 iter_final_bias = iterative_ekf->getGyroBias();
    
    printf("True Gyro Bias:      [%.4f, %.4f, %.4f]\n", 
           true_bias.x, true_bias.y, true_bias.z);
    printf("Standard EKF Bias:   [%.4f, %.4f, %.4f]\n", 
           std_final_bias.x, std_final_bias.y, std_final_bias.z);
    printf("Iterative EKF Bias:  [%.4f, %.4f, %.4f]\n", 
           iter_final_bias.x, iter_final_bias.y, iter_final_bias.z);
    
    // Calculate bias estimation errors
    Vector3 std_error = std_final_bias - true_bias;
    Vector3 iter_error = iter_final_bias - true_bias;
    
    printf("\nBias Estimation Error:\n");
    printf("Standard EKF:  %.5f (RMS)\n", std_error.norm());
    printf("Iterative EKF: %.5f (RMS)\n", iter_error.norm());
    
    // Clean up
    delete standard_ekf;
    delete iterative_ekf;
}

int main() {
    stdio_init_all();
    
    sleep_ms(2000);
    printf("Pico Iterative EKF Example\n");
    printf("==========================\n\n");
    
    // Initialize random seed
    srand(time_us_32());
    
    // Example 1: Basic Iterative EKF usage
    printf("Example 1: Basic Iterative EKF Usage\n");
    printf("------------------------------------\n");
    
    ExtendedKalmanFilter* ekf = createEKF(EKFType::Iterative);
    
    EKFConfig config;
    config.gyroNoise = 0.01f;
    config.gyroBiasNoise = 0.0001f;
    config.accelNoise = 0.1f;
    config.magNoise = 0.1f;
    config.gravityRef = Vector3(0, 0, 9.81f);
    config.magRef = Vector3(1, 0, 0);
    config.useMagnetometer = true;
    
    ekf->init(config);
    printf("Iterative EKF initialized\n\n");
    
    // Simulate with known bias
    Vector3 true_bias(0.01f, -0.005f, 0.02f);
    float time = 0.0f;
    const int warmup_iterations = 500;
    
    printf("Running Iterative EKF with simulated IMU data...\n");
    printf("Warming up filter for bias estimation...\n\n");
    
    // Warmup phase
    for (int i = 0; i < warmup_iterations; i++) {
        IMUMeasurement meas;
        simulate_challenging_imu_data(meas, time, true_bias, 1.0f);
        ekf->update(meas);
        time += meas.dt;
    }
    
    printf("Time(s), Roll(deg), Pitch(deg), Yaw(deg), ");
    printf("GyroBias_X, GyroBias_Y, GyroBias_Z\n");
    
    // Main loop with output
    const int main_iterations = 1000;
    for (int i = 0; i < main_iterations; i++) {
        IMUMeasurement meas;
        simulate_challenging_imu_data(meas, time, true_bias, 1.0f);
        
        ekf->update(meas);
        
        if (i % 100 == 0) {
            float roll, pitch, yaw;
            ekf->getEulerAngles(roll, pitch, yaw);
            
            Vector3 gyroBias = ekf->getGyroBias();
            
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
    
    printf("\nIterative EKF example completed!\n");
    
    // Clean up
    delete ekf;
    
    sleep_ms(2000);
    
    // Example 2: Performance comparison
    compare_ekf_performance();
    
    printf("\n\nAll examples completed!\n");
    
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}