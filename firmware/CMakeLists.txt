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

# Create the main executable
add_executable(picomm_imu
    src/main.cpp
)

# Link libraries
target_link_libraries(picomm_imu
    pico_stdlib
    pico_icm20948
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(picomm_imu)

# Enable USB output, disable UART output
pico_enable_stdio_usb(picomm_imu 1)
pico_enable_stdio_uart(picomm_imu 0)

