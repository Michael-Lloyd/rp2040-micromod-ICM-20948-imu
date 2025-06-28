cmake_minimum_required(VERSION 3.13...3.27)

# Board must be set before import and project definition 
set(PICO_BOARD sparkfun_micromod)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(picomm_imu)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()


# Add library subdirectories
add_subdirectory(lib/pico_icm20948)
add_subdirectory(lib/pico_ekf)

# Create the main executable
add_executable(picomm_imu
    src/main.cpp
    src/imu_collector.cpp
    src/ekf_processor.cpp
)

# Link libraries
target_link_libraries(picomm_imu
    pico_stdlib
    pico_icm20948
    pico_ekf
    pico_multicore
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(picomm_imu)

# Enable USB output, disable UART output
pico_enable_stdio_usb(picomm_imu 1)
pico_enable_stdio_uart(picomm_imu 0)

# Add test executable for StandardEKF
add_executable(test_standard_ekf
    test/standardEKF/main.cpp
    test/standardEKF/imu_collector.cpp
    test/standardEKF/ekf_processor.cpp
)

# Link libraries for test
target_link_libraries(test_standard_ekf
    pico_stdlib
    pico_icm20948
    pico_ekf
    pico_multicore
)

# Create map/bin/hex/uf2 files for test
pico_add_extra_outputs(test_standard_ekf)

# Enable USB output, disable UART output for test
pico_enable_stdio_usb(test_standard_ekf 1)
pico_enable_stdio_uart(test_standard_ekf 0)

