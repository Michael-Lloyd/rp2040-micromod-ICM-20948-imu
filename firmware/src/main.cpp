#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ICM_20948.h"

// I2C pins for Qwiic connector on MicroMod carrier board
#define I2C_SDA_PIN 4  // MicroMod Pin 13 -> RP2040 GPIO4
#define I2C_SCL_PIN 5  // MicroMod Pin 15 -> RP2040 GPIO5

// Create ICM-20948 object
ICM_20948_I2C myICM;

int main() {
        // Initialize stdio
        stdio_init_all();
    
        // Wait for USB serial connection (optional, but helpful for debugging)
        sleep_ms(3000);
    
        printf("\n");
        printf("=====================================\n");
        printf("ICM-20948 IMU - MicroMod RP2040\n");
        printf("=====================================\n");
        printf("Using Qwiic I2C connector\n");
    
        // Initialize I2C at 400kHz (standard fast mode for Qwiic devices)
        i2c_init(i2c_default, 400 * 1000);
        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    
        // Note: The MicroMod carrier board has built-in pull-ups on the I2C lines,
        // but we'll enable the internal ones as well for extra reliability
        gpio_pull_up(I2C_SDA_PIN);
        gpio_pull_up(I2C_SCL_PIN);
    
        printf("I2C initialized:\n");
        printf("  - Speed: 400 kHz\n");
        printf("  - SDA: GPIO%d (MicroMod Pin 13)\n", I2C_SDA_PIN);
        printf("  - SCL: GPIO%d (MicroMod Pin 15)\n", I2C_SCL_PIN);
    
        // Initialize ICM-20948
        printf("\nInitializing ICM-20948...\n");
    
        // Most ICM-20948 breakout boards default to address 0x69
        // Try 0x69 first, then 0x68 if that fails
        ICM_20948_Status_e status = myICM.begin(i2c_default, ICM_20948_I2C_ADDR_AD1);
    
        if (status != ICM_20948_Stat_Ok) {
                printf("Failed with address 0x69. Trying 0x68...\n");
                status = myICM.begin(i2c_default, ICM_20948_I2C_ADDR_AD0);
        
                if (status != ICM_20948_Stat_Ok) {
                        printf("\nERROR: Failed to initialize ICM-20948!\n");
                        printf("Status: %s\n", myICM.statusString(status));
                        printf("\nPlease check:\n");
                        printf("1. ICM-20948 is connected to Qwiic connector\n");
                        printf("2. Qwiic cable is properly seated\n");
                        printf("3. Board has power\n");
            
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
        }
    
        printf("SUCCESS! ICM-20948 initialized\n");
        printf("WHO_AM_I register: 0x%02X\n", myICM.getWhoAmI());
    
        // Configure sample rates
        ICM_20948_smplrt_t sampleRate;
        sampleRate.g = 19;  // Gyro: 1125Hz / (1 + 19) = 56.25Hz
        sampleRate.a = 19;  // Accel: 1125Hz / (1 + 19) = 56.25Hz
        myICM.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, sampleRate);
    
        printf("\nSensor configuration:\n");
        printf("  - Accelerometer: 56.25 Hz\n");
        printf("  - Gyroscope: 56.25 Hz\n");
        printf("  - Magnetometer: ~100 Hz\n");
    
        // Main loop
        printf("\nStarting sensor readings...\n");
        printf("=====================================\n\n");
    
        uint32_t readingCount = 0;
        absolute_time_t lastPrintTime = get_absolute_time();
    
        // Setup onboard LED for activity indication
        const uint LED_PIN = PICO_DEFAULT_LED_PIN;
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
    
        while (true) {
                // Check if new data is available
                if (myICM.dataReady()) {
                        // Toggle LED to show activity
                        gpio_put(LED_PIN, readingCount & 0x20);
            
                        // Read all sensor data
                        myICM.getAGMT();
            
                        // Print data every 200ms to avoid console flooding
                        if (absolute_time_diff_us(lastPrintTime, get_absolute_time()) >= 200000) {
                                lastPrintTime = get_absolute_time();
                
                                printf("Reading #%lu\n", readingCount);
                                printf(" Accel (mg):  X=%8.1f  Y=%8.1f  Z=%8.1f\n", 
                       myICM.accX(), myICM.accY(), myICM.accZ());
                                printf(" Gyro (dps):  X=%8.1f  Y=%8.1f  Z=%8.1f\n", 
                       myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
                                printf(" Mag (uT):    X=%8.1f  Y=%8.1f  Z=%8.1f\n", 
                       myICM.magX(), myICM.magY(), myICM.magZ());
                                printf(" Temp (C):    %.2f\n\n", myICM.temp());
                        }
            
                        readingCount++;
                }
        
                // Small delay to prevent I2C bus saturation
                sleep_us(500);
        }
    
        return 0;
}