#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/i2c.h"
#include "ICM_20948.h"
#include "imu_collector.h"
#include "ekf_processor.h"

// I2C pins for Qwiic connector on MicroMod carrier board
#define I2C_SDA_PIN 4  // MicroMod Pin 13 -> RP2040 GPIO4
#define I2C_SCL_PIN 5  // MicroMod Pin 15 -> RP2040 GPIO5

// Global objects
ICM_20948_I2C myICM;
mutex_t dataMutex;
IMUData sharedIMUData;

// Initialize I2C and ICM-20948
bool initializeHardware() {
    // Initialize I2C at 400kHz (standard fast mode for Qwiic devices)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    
    // Enable internal pull-ups
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    printf("I2C initialized:\n");
    printf("  - Speed: 400 kHz\n");
    printf("  - SDA: GPIO%d \n", I2C_SDA_PIN);
    printf("  - SCL: GPIO%d \n", I2C_SCL_PIN);
    
    // Initialize ICM-20948
    printf("\nInitializing ICM-20948...\n");
    
    // Try 0x69 first, then 0x68 if that fails
    ICM_20948_Status_e status = myICM.begin(i2c_default, ICM_20948_I2C_ADDR_AD1);
    
    if (status != ICM_20948_Stat_Ok) {
        printf("Failed with address 0x69. Trying 0x68...\n");
        status = myICM.begin(i2c_default, ICM_20948_I2C_ADDR_AD0);
        
        if (status != ICM_20948_Stat_Ok) {
            printf("\nERROR: Failed to initialize ICM-20948!\n");
            printf("Status: %s\n", myICM.statusString(status));
            return false;
        }
    }
    
    printf("SUCCESS! ICM-20948 initialized\n");
    printf("WHO_AM_I register: 0x%02X\n", myICM.getWhoAmI());
    
    return true;
}

// Core 1 entry point - runs EKF processing
void core1_entry() {
    // Create and run EKF processor
    EKFProcessor processor(&dataMutex, &sharedIMUData);
    ekf_processor_thread();
}

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Wait for USB serial connection (optional, but helpful for debugging)
    sleep_ms(3000);
    
    printf("\n");
    printf("=====================================\n");
    printf("IMU + EKF MULTI-THREADED SYSTEM\n");
    printf("=====================================\n");
    
    // Initialize mutex for thread-safe data sharing
    mutex_init(&dataMutex);
    
    // Initialize hardware
    if (!initializeHardware()) {
        // Blink LED to indicate error
        const uint LED_PIN = PICO_DEFAULT_LED_PIN;
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        while (1) {
            gpio_put(LED_PIN, 1);
            sleep_ms(250);
            gpio_put(LED_PIN, 0);
            sleep_ms(250);
        }
    }
    
    printf("\nSystem configuration:\n");
    printf("  - Core 0: IMU data collection (56.25 Hz)\n");
    printf("  - Core 1: EKF processing (100 Hz)\n");
    printf("  - Thread-safe data sharing via mutex\n");
    
    // Launch Core 1 for EKF processing
    printf("\nStarting EKF processor on Core 1...\n");
    multicore_launch_core1(core1_entry);
    
    // Small delay to let Core 1 initialize
    sleep_ms(100);
    
    // Create IMU collector and run on Core 0
    printf("Starting IMU collector on Core 0...\n");
    printf("=====================================\n\n");
    
    IMUCollector collector(&myICM, &dataMutex, &sharedIMUData);
    
    // Setup onboard LED for activity indication
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Run IMU collection on Core 0
    imu_collector_thread();
    
    return 0;
}