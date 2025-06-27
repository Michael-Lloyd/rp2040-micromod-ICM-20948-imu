/**
 * Example test program for ICM-20948 IMU with Raspberry Pi Pico
 * 
 * This example demonstrates basic usage of the ICM-20948 9-axis IMU
 * including accelerometer, gyroscope, and magnetometer readings.
 * 
 * Wiring:
 * - ICM-20948 SDA -> Pico GPIO 4 (or your chosen SDA pin)
 * - ICM-20948 SCL -> Pico GPIO 5 (or your chosen SCL pin)
 * - ICM-20948 VDD -> 3.3V
 * - ICM-20948 GND -> GND
 * - ICM-20948 AD0 -> GND (for I2C address 0x68) or 3.3V (for 0x69)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ICM_20948.h"

// I2C pins
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// Create ICM-20948 object
ICM_20948_I2C myICM;

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Wait a bit for USB serial to connect (optional)
    sleep_ms(2000);
    
    printf("ICM-20948 Test Program\n");
    printf("======================\n");
    
    // Initialize I2C
    i2c_init(i2c_default, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    printf("I2C initialized at 400kHz\n");
    printf("SDA: GPIO%d, SCL: GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Enable debug output (optional)
    myICM.enableDebugging();
    
    // Initialize ICM-20948
    printf("\nInitializing ICM-20948...\n");
    ICM_20948_Status_e status = myICM.begin(i2c_default, ICM_20948_I2C_ADDR_AD0);
    
    if (status != ICM_20948_Stat_Ok) {
        printf("Failed to initialize ICM-20948! Status: %s\n", myICM.statusString(status));
        printf("Please check your wiring and I2C address.\n");
        
        // Try the other I2C address
        printf("\nTrying alternate I2C address (0x69)...\n");
        status = myICM.begin(i2c_default, ICM_20948_I2C_ADDR_AD1);
        
        if (status != ICM_20948_Stat_Ok) {
            printf("Failed with alternate address too. Exiting.\n");
            while (1) {
                sleep_ms(1000);
            }
        }
    }
    
    printf("ICM-20948 initialized successfully!\n");
    printf("WHO_AM_I: 0x%02X\n", myICM.getWhoAmI());
    
    // Optional: Configure sensor settings
    // Set sample rate divider for accelerometer and gyroscope
    ICM_20948_smplrt_t mySmplrt;
    mySmplrt.g = 19; // Gyro sample rate = 1125Hz / (1 + 19) = 56.25Hz
    mySmplrt.a = 19; // Accel sample rate = 1125Hz / (1 + 19) = 56.25Hz
    myICM.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, mySmplrt);
    
    // Main loop - read and display sensor data
    printf("\nStarting sensor readings...\n");
    printf("Press Ctrl+C to exit\n\n");
    
    uint32_t loopCount = 0;
    absolute_time_t lastPrintTime = get_absolute_time();
    
    while (true) {
        // Check if data is ready
        if (myICM.dataReady()) {
            // Read all sensor data
            myICM.getAGMT(); // Updates the sensor data
            
            // Only print every 100ms to avoid flooding the console
            if (absolute_time_diff_us(lastPrintTime, get_absolute_time()) >= 100000) {
                lastPrintTime = get_absolute_time();
                
                printf("--- Reading #%lu ---\n", loopCount++);
                
                // Print accelerometer data (milli-g)
                printf("Accel: X=%7.1f Y=%7.1f Z=%7.1f mg\n", 
                       myICM.accX(), myICM.accY(), myICM.accZ());
                
                // Print gyroscope data (degrees per second)
                printf("Gyro:  X=%7.1f Y=%7.1f Z=%7.1f dps\n", 
                       myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
                
                // Print magnetometer data (micro Tesla)
                printf("Mag:   X=%7.1f Y=%7.1f Z=%7.1f uT\n", 
                       myICM.magX(), myICM.magY(), myICM.magZ());
                
                // Print temperature (Celsius)
                printf("Temp:  %.2f C\n", myICM.temp());
                
                printf("\n");
            }
        }
        
        // Small delay to prevent overwhelming the I2C bus
        sleep_ms(1);
    }
    
    return 0;
}