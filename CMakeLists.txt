cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME infrabot_firmware)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(FW_NETBOOT "Build the signed A/B application" ON)
option(FW_STANDALONE "Explicit standalone image at 0x08000000 (non-netboot boards only)" OFF)
if((NOT FW_NETBOOT AND NOT FW_STANDALONE) OR (FW_NETBOOT AND FW_STANDALONE))
    message(FATAL_ERROR "Choose A/B (FW_NETBOOT=ON) or the explicit StandaloneDebug preset for non-netboot boards.")
endif()
if(FW_STANDALONE AND (CMAKE_BINARY_DIR STREQUAL "${CMAKE_SOURCE_DIR}/build/Debug" OR CMAKE_BINARY_DIR STREQUAL "${CMAKE_SOURCE_DIR}/build/Release"))
    message(FATAL_ERROR "Use the separate StandaloneDebug directory; Debug and Release are reserved for A/B.")
endif()
if(FW_NETBOOT)
    # Both A/B images must fit a slot, including in Debug. Keep DWARF symbols
    # while matching the optimized timing of the 1 kHz control loop.
    add_compile_options("$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C,CXX>>:-Os>")
endif()

# Pinned middleware, with an optional external STM32CubeH7 package override.
set(STM32CUBE_ROOT "${CMAKE_SOURCE_DIR}/third_party/STM32CubeH7"
    CACHE PATH "STM32CubeH7 package root")
add_compile_options("-ffile-prefix-map=${CMAKE_SOURCE_DIR}/=")
if(NOT STM32CUBE_ROOT STREQUAL "${CMAKE_SOURCE_DIR}/third_party/STM32CubeH7")
    add_compile_options("-ffile-prefix-map=${STM32CUBE_ROOT}/=third_party/STM32CubeH7/")
endif()
set(LWIP_ROOT "${STM32CUBE_ROOT}/Middlewares/Third_Party/LwIP")
set(FREERTOS_ROOT "${STM32CUBE_ROOT}/Middlewares/Third_Party/FreeRTOS/Source")
set(ETH_COMPONENT_ROOT "${STM32CUBE_ROOT}/Drivers/BSP/Components/lan8742")
file(GLOB LWIP_CORE_SOURCES CONFIGURE_DEPENDS
    "${LWIP_ROOT}/src/core/*.c"
    "${LWIP_ROOT}/src/core/ipv4/*.c"
    "${LWIP_ROOT}/src/netif/ethernet.c")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_library(${CMAKE_PROJECT_NAME} STATIC)

# Add STM32CubeMX generated sources
include(boards/fire-h743-challenger-v2/board.cmake)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    drivers/damiao/dm_motor.c
    drivers/damiao/dm_protocol.c
    app/app_state.c
    drivers/can/bus_state.c
    control/control_stats.c
    drivers/damiao/dm_runtime.c
    drivers/net/eth_state.c
    app/uart_tx_state.c
    app/app_persist.c
    app/app_tasks.c
    control/arm_motion.c
    control/infrabot_ik_mcu.c
    control/app_motion.c
    control/app_control_mode.c
    control/robot_controller.c
    drivers/damiao/app_damiao.c
    control/app_enable.c
    control/app_arm_power.c
    control/app_gripper.c
    control/app_safety.c
    control/app_impedance.c
    control/arm_impedance.c
    control/arm_admittance.c
    control/arm_admittance_jitter.c
    control/app_admittance.c
    control/app_admittance_parameters.c
    control/app_gravity.c
    control/app_local_control.c
    app/app_panel.c
    control/arm_gravity.cpp
    app/app_uart.c
    app/app_log.c
    app/app_log_task.c
    drivers/can/app_fdcan.c
    app/app_startup.c
    app/app_rtos.c
    drivers/display/app_sdram.c
    drivers/display/app_lcd.c
    drivers/display/app_lcd_panel.c
    drivers/display/app_touch.c
    third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ltdc.c
    third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sdram.c
    third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_fmc.c
    drivers/net/ethernetif.c
    drivers/net/app_info.c
    drivers/net/app_ethernet.c
    app/freertos_heap.c
    app/lwip_heap.c
    ${LWIP_CORE_SOURCES}
    ${ETH_COMPONENT_ROOT}/lan8742.c
    ${FREERTOS_ROOT}/tasks.c
    ${FREERTOS_ROOT}/queue.c
    ${FREERTOS_ROOT}/list.c
    ${FREERTOS_ROOT}/portable/GCC/ARM_CM7/r0p1/port.c
    ${FREERTOS_ROOT}/portable/MemMang/heap_4.c
)

# The RT Cartesian IK runs inside the 1 kHz control tick with a 500 us
# budget; at the Debug -O0 level it can overrun and silently hold position.
set_source_files_properties(control/infrabot_ik_mcu.c PROPERTIES COMPILE_OPTIONS "-O2")
set_source_files_properties(control/arm_gravity.cpp control/app_gravity.c PROPERTIES COMPILE_OPTIONS "-O2")
# The LCD 3D page rasterises ~6700 triangles per frame; -O0 would cut its
# frame rate several times, and -ffast-math lets fminf/rounding inline (it
# only draws pixels, in the lowest-priority task).
set_source_files_properties(ui/app_lcd_3d.c PROPERTIES COMPILE_OPTIONS "-O2;-ffast-math")

# A/B switch for on-board measurements of cached scratch versus SDRAM
# stores.  Both modes use the same mesh, rasterizer and timing readouts.
option(APP_LCD_SCRATCH_CACHE "Cache the LCD off-screen drawing buffer" ON)
option(APP_LCD_PARTIAL_REFRESH "Update only old/new 3D scene bounds" ON)
option(APP_LCD_FAST_BUFFER "Draw 3D updates in 128 KiB AXI SRAM tiles" ON)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    APP_LCD_SCRATCH_CACHE=$<BOOL:${APP_LCD_SCRATCH_CACHE}>
    APP_LCD_PARTIAL_REFRESH=$<BOOL:${APP_LCD_PARTIAL_REFRESH}>
    APP_LCD_FAST_BUFFER=$<BOOL:${APP_LCD_FAST_BUFFER}>)

# Add include paths.  The UDP protocol header is protocol/infrabot/protocol.h,
# this repository's copy of the PC's (see docs/control-modes.md).
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_ROOT}/include
    ${FREERTOS_ROOT}/portable/GCC/ARM_CM7/r0p1
    ${LWIP_ROOT}/src/include
    ${LWIP_ROOT}/system
    ${ETH_COMPONENT_ROOT}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    # LCD HAL modules (app_sdram.c / app_lcd.c).  Enabled here rather than in
    # the generated stm32h7xx_hal_conf.h so a CubeMX regeneration keeps them.
    HAL_LTDC_MODULE_ENABLED
    HAL_SDRAM_MODULE_ENABLED
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Panel detection and coordinate validation are shared with host tests.
set_source_files_properties(drivers/display/app_lcd_panel.c PROPERTIES COMPILE_OPTIONS "-O2")

set_source_files_properties(ui/app_lcd_boot.c PROPERTIES COMPILE_OPTIONS "-O2")
# Relink when startup placement guards or memory regions change.
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/STM32H743xx_FLASH.ld")

# Identify the firmware actually built, including uncommitted source changes.
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated")
add_custom_target(firmware_build_info
    COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${CMAKE_BINARY_DIR}/generated/app_build_info.h
        -P ${CMAKE_SOURCE_DIR}/cmake/GenerateBuildInfo.cmake
    BYPRODUCTS "${CMAKE_BINARY_DIR}/generated/app_build_info.h")
add_dependencies(${CMAKE_PROJECT_NAME} firmware_build_info)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_BINARY_DIR}/generated")

set_source_files_properties(control/arm_admittance_jitter.c control/app_admittance_parameters.c control/arm_admittance.c control/app_admittance.c control/arm_impedance.c control/app_impedance.c PROPERTIES COMPILE_OPTIONS "-O2")

