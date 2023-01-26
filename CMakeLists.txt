cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(e32_lib_project)


# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_library(lora_e32 STATIC LoRa_E32.cpp)
target_link_libraries(lora_e32 pico_stdlib)

target_include_directories(lora_e32 INTERFACE .)

