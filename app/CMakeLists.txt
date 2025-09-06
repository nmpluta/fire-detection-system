#-------------------------------------------------------------------------------
# Zephyr Fire Detection System
#
# Copyright (c) 2025 Natalia Pluta
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE 
    src/main.c
    src/modules/sensor/sensor_module.c
    src/modules/controller/controller_module.c
)

# Add include directories for modules
target_include_directories(app PRIVATE src)
