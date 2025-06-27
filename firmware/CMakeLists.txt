cmake_minimum_required(VERSION 3.13...3.27)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(picomm_imu)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

