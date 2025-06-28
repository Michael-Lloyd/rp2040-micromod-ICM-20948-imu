# ICM-20948 Library for Raspberry Pi Pico

This is a C++ library for the ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer) 
designed specifically for use with the Raspberry Pi Pico and the pico-sdk. 

## Features

- I2C interface support (SPI removed for simplicity)
- Full access to accelerometer, gyroscope, and magnetometer data
- Temperature sensor reading
- Configurable sample rates and full-scale ranges
- Digital Low Pass Filter (DLPF) configuration
- DMP (Digital Motion Processor) support (optional)

## Library Structure

```
pico_icm20948/
├── src/
│   ├── ICM_20948.h          # Main C++ header
│   ├── ICM_20948.cpp        # Main C++ implementation
│   └── util/
│       ├── ICM_20948_C.h    # C backbone header
│       ├── ICM_20948_C.c    # C backbone implementation
│       ├── ICM_20948_REGISTERS.h     # Register definitions
│       ├── ICM_20948_ENUMERATIONS.h  # Enumeration definitions
│       ├── ICM_20948_DMP.h           # DMP definitions
│       ├── AK09916_REGISTERS.h       # Magnetometer registers
│       ├── AK09916_ENUMERATIONS.h    # Magnetometer enumerations
│       └── icm20948_img.dmp3a.h      # DMP firmware image
├── example/
│   ├── icm20948_test.cpp    # Example test program
│   └── CMakeLists.txt       # CMake build file
└── README.md               # This file
```

## Usage

### Basic Example

```cpp
#include "ICM_20948.h"
#include "hardware/i2c.h"

// Create ICM-20948 object
ICM_20948_I2C myICM;

int main() {
    // Initialize I2C
    i2c_init(i2c_default, 400 * 1000); // 400kHz
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);
    
    // Initialize ICM-20948
    myICM.begin(i2c_default, ICM_20948_I2C_ADDR_AD0);
    
    // Read sensor data
    while (true) {
        if (myICM.dataReady()) {
            myICM.getAGMT(); // Update all sensor readings
            
            printf("Accel: X=%.1f Y=%.1f Z=%.1f mg\n", 
                   myICM.accX(), myICM.accY(), myICM.accZ());
            printf("Gyro: X=%.1f Y=%.1f Z=%.1f dps\n", 
                   myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
            printf("Mag: X=%.1f Y=%.1f Z=%.1f uT\n", 
                   myICM.magX(), myICM.magY(), myICM.magZ());
        }
        sleep_ms(100);
    }
}
```

## API Reference

### Initialization

```cpp
ICM_20948_Status_e begin(i2c_inst_t *i2cPort, uint8_t addr)
```
Initialize the ICM-20948 with specified I2C port and address.

### Reading Sensor Data

```cpp
bool dataReady()              // Check if new data is available
ICM_20948_AGMT_t getAGMT()    // Read all sensors
float accX/Y/Z()              // Get accelerometer data (milli-g)
float gyrX/Y/Z()              // Get gyroscope data (degrees/sec)
float magX/Y/Z()              // Get magnetometer data (micro Tesla)
float temp()                  // Get temperature (Celsius)
```

### Configuration

```cpp
setSampleRate()               // Set sample rate
setFullScale()                // Set full scale range
setDLPFcfg()                  // Configure digital low pass filter
enableDLPF()                  // Enable/disable DLPF
```

### Debug

```cpp
enableDebugging()             // Enable debug printf output
disableDebugging()            // Disable debug output
```

## DMP Support

The Digital Motion Processor (DMP) is supported but disabled by default. To enable DMP support:

1. Uncomment the following line in `src/util/ICM_20948_C.h`:
   ```c
   #define ICM_20948_USE_DMP
   ```

2. Rebuild your project

Note: DMP firmware requires approximately 14KB of program memory.

## License

This library is based on the SparkFun ICM-20948 Arduino Library and maintains compatibility with its license terms. Please refer to the original library for complete license information.
