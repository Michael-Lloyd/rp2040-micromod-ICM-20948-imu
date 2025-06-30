cmake_minimum_required(VERSION 3.13)

# Create the pico_icm20948 library
add_library(pico_icm20948 STATIC
    src/ICM_20948.cpp
    src/util/ICM_20948_C.c
)

# Include directories
target_include_directories(pico_icm20948 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/util
)

# Link with required libraries
target_link_libraries(pico_icm20948 PUBLIC
    pico_stdlib
    hardware_i2c
    hardware_spi
)

# Set compile options for performance
target_compile_options(pico_icm20948 PRIVATE
    -O2
    -ffast-math
)

# Optional: Enable debug output
if(PICO_ICM20948_DEBUG)
    target_compile_definitions(pico_icm20948 PUBLIC PICO_ICM20948_DEBUG=1)
endif()