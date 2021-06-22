# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

# include(utils/cmake/utils.cmake)

# Needed for colorful output
zephyr_compile_options(-fdiagnostics-color=always)

# NORDIC SDK APP START

# set(INCLUDES
    # edge-impulse-custom/ingestion-sdk-c
    # edge-impulse-custom/ingestion-sdk-platform/NordicSemi-nrf52
    # edge-impulse-custom/ingestion-sdk-platform/NordicSemi-nrf52/sensors
    # edge-impulse-custom/mbedtls_hmac_sha256_sw
    # edge-impulse-custom/QCBOR/inc
    # edge-impulse-custom/QCBOR/src
    # edge-impulse-custom/repl
    # )
# include_directories(${INCLUDES})

# RECURSIVE_FIND_FILE(SOURCE_FILES "edge-impulse-custom/ingestion-sdk-platform/NordicSemi-nrf52" "*.cpp")
# RECURSIVE_FIND_FILE(REPL_FILES "edge-impulse-custom/repl" "*.cpp")
# RECURSIVE_FIND_FILE(INGESTION_FILES "edge-impulse-custom/ingestion-sdk-c/" "*.cpp")
# RECURSIVE_FIND_FILE(QCBOR_FILES "edge-impulse-custom/QCBOR/src" "*.c")
# RECURSIVE_FIND_FILE(MBEDTLS_FILES "edge-impulse-custom/mbedtls_hmac_sha256_sw/mbedtls/src" "*.c")

# list(APPEND SOURCE_FILES ${REPL_FILES})
# list(APPEND SOURCE_FILES ${INGESTION_FILES})
# list(APPEND SOURCE_FILES ${QCBOR_FILES})
# list(APPEND SOURCE_FILES ${MBEDTLS_FILES})

# # add all sources to the project
# target_sources(app PRIVATE ${SOURCE_FILES})

target_include_directories(app PRIVATE src/include)
target_sources(app PRIVATE src/ei_microphone.cpp)
target_sources(app PRIVATE src/main.c)

# NORDIC SDK APP END
