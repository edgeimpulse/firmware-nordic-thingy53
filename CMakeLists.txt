#
# Copyright (c) 2022 Edge Impulse
#
#

cmake_minimum_required(VERSION 3.13.1)

set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/pm_static_${BOARD}.yml)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/vm3011 )

########################################################################## 
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("firmware-nordic-thingy53-internal"
          VERSION 0.1)
##########################################################################

add_subdirectory(ei-model/edge-impulse-sdk/cmake/zephyr)

# Needed for colorful output
zephyr_compile_options(-fdiagnostics-color=always)

# Use hardware acceleration for DSP and Neural Network code
# You'll need to disable these on non-Arm cores
add_definitions(-DEIDSP_USE_CMSIS_DSP=1
                -DEIDSP_LOAD_CMSIS_DSP_SOURCES=1
                -DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
                -DEIDSP_QUANTIZE_FILTERBANK=0
                -DARM_MATH_LOOPUNROLL
                -DEI_SENSOR_AQ_STREAM=FILE
                -DMBEDTLS_PLATFORM_ZEROIZE_ALT
                )

# Edge impulse SDK include directories
set(INCLUDES
    .
    drivers
    ei-model
    ei-model/tflite-model
    ei-model/model-parameters
    src
    src/libs/mbedtls_hmac_sha256_sw
    src/libs/QCBOR/inc
    src/libs/QCBOR/src
    src/libs
    )
include_directories(${INCLUDES})

RECURSIVE_FIND_FILE(MODEL_FILES "ei-model/tflite-model" "*.cpp")
RECURSIVE_FIND_FILE(BME68X_FILES "drivers/bme68x" "*.c")
RECURSIVE_FIND_FILE(FIRMWARE_SDK_FILES "firmware-sdk" "*.cpp")
RECURSIVE_FIND_FILE(LIB_QCBOR_FILES "src/libs/QCBOR/src" "*.c")
RECURSIVE_FIND_FILE(LIB_MBEDTLS_FILES "src/libs/mbedtls_hmac_sha256_sw/mbedtls/src" "*.c")
RECURSIVE_FIND_FILE(LIB_SENSOR_AQ_MBEDTLS_FILES "src/libs/sensor_aq_mbedtls" "*.cpp")
RECURSIVE_FIND_FILE(SENSORS_FILES "src/sensors" "*.cpp")
RECURSIVE_FIND_FILE(INFERENCE_FILES "src/inference" "*.cpp")

list(APPEND SOURCE_FILES ${MODEL_FILES})
list(APPEND SOURCE_FILES ${BME68X_FILES})
list(APPEND SOURCE_FILES ${FIRMWARE_SDK_FILES})
list(APPEND SOURCE_FILES ${LIB_QCBOR_FILES})
list(APPEND SOURCE_FILES ${LIB_MBEDTLS_FILES})
list(APPEND SOURCE_FILES ${LIB_SENSOR_AQ_MBEDTLS_FILES})
list(APPEND SOURCE_FILES ${SENSORS_FILES})
list(APPEND SOURCE_FILES ${INFERENCE_FILES})

# add all sources to the project
target_sources(app PRIVATE ${SOURCE_FILES})
target_sources(app PRIVATE src/ble/ei_ble_com.cpp)
target_sources(app PRIVATE src/ble/nus.c)
target_sources(app PRIVATE src/ble/ble_nus.cpp)
target_sources(app PRIVATE src/ei_base64_encode.cpp)
target_sources(app PRIVATE src/ei_at_handlers.cpp)
target_sources(app PRIVATE src/ei_device_thingy53.cpp)
target_sources(app PRIVATE src/ei_sampler.cpp)
target_sources(app PRIVATE src/flash_memory.cpp)
target_sources(app PRIVATE src/main.cpp)

# Preinitialization related to Thingy:53 DFU
target_sources_ifdef(CONFIG_BOARD_THINGY53_NRF5340_CPUAPP app PRIVATE
  boards/thingy53.c
)
