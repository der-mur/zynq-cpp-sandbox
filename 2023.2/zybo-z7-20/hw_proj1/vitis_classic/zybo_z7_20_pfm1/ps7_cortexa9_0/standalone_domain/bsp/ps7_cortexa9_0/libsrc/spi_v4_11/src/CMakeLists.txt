# Copyright (C) 2023 Advanced Micro Devices, Inc.  All rights reserved.
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.15)
project(spi)

find_package(common)
collector_create (PROJECT_LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}")
collector_create (PROJECT_LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_BINARY_DIR}/include)
collect (PROJECT_LIB_SOURCES xspi.c)
collect (PROJECT_LIB_HEADERS xspi.h)
collect (PROJECT_LIB_SOURCES xspi_g.c)
collect (PROJECT_LIB_HEADERS xspi_i.h)
collect (PROJECT_LIB_HEADERS xspi_l.h)
collect (PROJECT_LIB_SOURCES xspi_options.c)
collect (PROJECT_LIB_SOURCES xspi_selftest.c)
collect (PROJECT_LIB_SOURCES xspi_sinit.c)
collect (PROJECT_LIB_SOURCES xspi_stats.c)
collector_list (_sources PROJECT_LIB_SOURCES)
collector_list (_headers PROJECT_LIB_HEADERS)
file(COPY ${_headers} DESTINATION ${CMAKE_BINARY_DIR}/include)
add_library(spi STATIC ${_sources})
set_target_properties(spi PROPERTIES LINKER_LANGUAGE C)
