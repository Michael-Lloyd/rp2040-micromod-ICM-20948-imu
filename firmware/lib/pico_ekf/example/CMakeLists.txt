cmake_minimum_required(VERSION 3.13)

# Initialize the SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ekf_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Create the example executable
add_executable(ekf_example
    ekf_example.cpp
)

# Link libraries
target_link_libraries(ekf_example
    pico_stdlib
    pico_ekf
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(ekf_example 1)
pico_enable_stdio_uart(ekf_example 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(ekf_example)

# Create the iterative EKF example executable
add_executable(iterative_ekf_example
    iterative_ekf_example.cpp
)

# Link libraries
target_link_libraries(iterative_ekf_example
    pico_stdlib
    pico_ekf
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(iterative_ekf_example 1)
pico_enable_stdio_uart(iterative_ekf_example 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(iterative_ekf_example)