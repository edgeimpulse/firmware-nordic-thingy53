# /* The Clear BSD License
#  *
#  * Copyright (c) 2025 EdgeImpulse Inc.
#  * All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted (subject to the limitations in the disclaimer
#  * below) provided that the following conditions are met:
#  *
#  *   * Redistributions of source code must retain the above copyright notice,
#  *   this list of conditions and the following disclaimer.
#  *
#  *   * Redistributions in binary form must reproduce the above copyright
#  *   notice, this list of conditions and the following disclaimer in the
#  *   documentation and/or other materials provided with the distribution.
#  *
#  *   * Neither the name of the copyright holder nor the names of its
#  *   contributors may be used to endorse or promote products derived from this
#  *   software without specific prior written permission.
#  *
#  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
#  * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#  * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#  * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#  * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  */

cmake_minimum_required(VERSION 3.13.1)

# If project is build with WiFi shiled using the command line
# west build -b thingy53_nrf5340_cpuapp -p -- -DSHIELD=nrf7002eb
# then the SHIELD variable is set to nrf7002eb and additional configuratio for project is added
if("${SHIELD}" STREQUAL "nrf7002eb")
    set(CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/shield-nrf7002eb.conf; ${CMAKE_CURRENT_SOURCE_DIR}/boards/thingy53_nrf5340_cpuapp.conf; ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf)
endif()

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
    src/libs
    )
include_directories(${INCLUDES})

RECURSIVE_FIND_FILE(MODEL_FILES "ei-model/tflite-model" "*.cpp")
RECURSIVE_FIND_FILE(BME68X_FILES "drivers/bme68x" "*.c")
RECURSIVE_FIND_FILE(FIRMWARE_SDK_FILES "firmware-sdk" "*.cpp")
RECURSIVE_FIND_FILE(FIRMWARE_SDK_C_FILES "firmware-sdk" "*.c")
RECURSIVE_FIND_FILE(LIB_MBEDTLS_FILES "src/libs/mbedtls_hmac_sha256_sw/mbedtls/src" "*.c")
RECURSIVE_FIND_FILE(SENSORS_FILES "src/sensors" "*.cpp")
RECURSIVE_FIND_FILE(INFERENCE_FILES "src/inference" "*.cpp")

list(APPEND SOURCE_FILES ${MODEL_FILES})
list(APPEND SOURCE_FILES ${BME68X_FILES})
list(APPEND SOURCE_FILES ${FIRMWARE_SDK_FILES})
list(APPEND SOURCE_FILES ${FIRMWARE_SDK_C_FILES})
list(APPEND SOURCE_FILES ${LIB_MBEDTLS_FILES})
list(APPEND SOURCE_FILES ${LIB_SENSOR_AQ_MBEDTLS_FILES})
list(APPEND SOURCE_FILES ${SENSORS_FILES})
list(APPEND SOURCE_FILES ${INFERENCE_FILES})

# add all sources to the project
target_sources(app PRIVATE ${SOURCE_FILES})
target_sources(app PRIVATE src/ble/ei_ble_com.cpp)
target_sources(app PRIVATE src/ble/nus.c)
target_sources(app PRIVATE src/ble/ble_nus.cpp)
if("${SHIELD}" STREQUAL "nrf7002eb")
    target_sources(app PRIVATE src/wifi/wifi.cpp)
    target_sources(app PRIVATE src/wifi/ei_ws_client.cpp)
endif()
target_sources(app PRIVATE src/ei_base64_encode.cpp)
target_sources(app PRIVATE src/ei_at_handlers.cpp)
target_sources(app PRIVATE src/ei_device_thingy53.cpp)
target_sources(app PRIVATE src/ei_sampler.cpp)
target_sources(app PRIVATE src/flash_memory.cpp)
target_sources(app PRIVATE src/main.cpp)
