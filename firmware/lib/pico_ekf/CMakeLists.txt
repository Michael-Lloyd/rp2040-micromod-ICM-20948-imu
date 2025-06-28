cmake_minimum_required(VERSION 3.13)

# Create the pico_ekf library
add_library(pico_ekf STATIC
    src/picoEKF.cpp
    src/StandardEKF.cpp
)

# Include directories
target_include_directories(pico_ekf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link with pico standard library
target_link_libraries(pico_ekf PUBLIC
    pico_stdlib
)

# Enable/disable debug output
if(PICO_EKF_DEBUG)
    target_compile_definitions(pico_ekf PUBLIC PICO_EKF_DEBUG=1)
endif()

# Set compile options for performance
target_compile_options(pico_ekf PRIVATE
    -O2
    -ffast-math
)

# Add example subdirectory if building standalone
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_subdirectory(example)
endif()