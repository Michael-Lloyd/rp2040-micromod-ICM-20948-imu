# Example CMakeLists.txt for ICM-20948 test program
cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(icm20948_example C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Add the ICM-20948 library source files
add_library(icm20948
    ../src/ICM_20948.cpp
    ../src/util/ICM_20948_C.c
)

# Include directories for the library
target_include_directories(icm20948 PUBLIC
    ../src
    ../src/util
)

# Add executable
add_executable(icm20948_test
    icm20948_test.cpp
)

# Link libraries
target_link_libraries(icm20948_test
    icm20948
    pico_stdlib
    hardware_i2c
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(icm20948_test 1)
pico_enable_stdio_uart(icm20948_test 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(icm20948_test)