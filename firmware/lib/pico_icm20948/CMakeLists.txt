add_library(pico_icm20948 INTERFACE)

target_sources(pico_icm20948 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/ICM_20948.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/util/ICM_20948_C.c
)

target_include_directories(pico_icm20948 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/util
)

target_link_libraries(pico_icm20948 INTERFACE
    pico_stdlib
    hardware_i2c
    hardware_spi
)