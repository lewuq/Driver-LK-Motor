# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xiao_stm32c5_lkm_gimbal)

target_include_directories(app PRIVATE ../include)
target_sources(app PRIVATE
  ../src/main.c
  ../src/lkm_motor.c
  ../src/gimbal.c
  ../src/touch.c
  ../src/round_ui.c
)
