# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
set(FW_SIGNING_KEY_FILE "" CACHE FILEPATH "Publisher Ed25519 private PEM (packager only)")
# Migrate only the former in-tree default in existing CMake build trees.
if(FW_PUBLIC_KEY_FILE STREQUAL "${CMAKE_SOURCE_DIR}/netboot/release.pub" AND
   NOT EXISTS "${FW_PUBLIC_KEY_FILE}")
    unset(FW_PUBLIC_KEY_FILE CACHE)
endif()
set(FW_PUBLIC_KEY_FILE "${CMAKE_SOURCE_DIR}/bootloader/release.pub" CACHE FILEPATH "Trusted Ed25519 public key (hex)")
file(READ "${FW_PUBLIC_KEY_FILE}" pub)
string(STRIP "${pub}" pub)
string(LENGTH "${pub}" keylen)
if(NOT keylen EQUAL 64 OR NOT pub MATCHES "^[0-9a-fA-F]+$")
  message(FATAL_ERROR "Invalid public key")
endif()
string(REGEX REPLACE "([0-9a-fA-F][0-9a-fA-F])" "0x\\1," initializer "${pub}")
file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/generated/fw_public.h" CONTENT "#define FW_PUBLIC_BYTES {@initializer@}\n" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${FW_PUBLIC_KEY_FILE}" "${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/STM32H743xx_FLASH.ld")
file(READ "${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/STM32H743xx_FLASH.ld" layout)
string(REPLACE "0x20000000, LENGTH = 128K" "0x20000060, LENGTH = 0x1ffa0" layout "${layout}")
string(REPLACE "0x8000000, LENGTH = 1920K" "0x08000000, LENGTH = 128K" bootlayout "${layout}")
string(REPLACE "0x8000000, LENGTH = 1920K" "0x08020400, LENGTH = 0xdfc00" applayout "${layout}")
string(REPLACE "*(.rodata*)" "*(EXCLUDE_FILE (*app_lcd_boot.c.obj) .rodata*)" applayout "${applayout}")
string(REPLACE "  /* Uninitialized data section */" "  .boot_graphics : { *app_lcd_boot.c.obj(.rodata*) } >FLASH\n  /* Uninitialized data section */" applayout "${applayout}")
string(REPLACE "  .text :" "  .fw_api 0x08000400 : { KEEP(*(.fw_api)) } >FLASH\n  .text :" bootlayout "${bootlayout}")
file(WRITE "${CMAKE_BINARY_DIR}/loader.ld" "${bootlayout}")
string(APPEND applayout "\nASSERT(ADDR(.isr_vector) == ORIGIN(FLASH), \"Bad application vector location\")\nASSERT(ORIGIN(FLASH) >= 0x08020400 && ORIGIN(FLASH) + LENGTH(FLASH) <= 0x081e0000, \"Application overlaps bootloader or config\")\n")
file(WRITE "${CMAKE_BINARY_DIR}/application.ld" "${applayout}")
string(REPLACE "-T \"${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/STM32H743xx_FLASH.ld\"" "" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
set(mbed "${STM32CUBE_ROOT}/Middlewares/Third_Party/mbedTLS")
set(common bootloader/challenge.c bootloader/service.c)
target_sources(infrabot_firmware PRIVATE ${common} bootloader/app_gate.c bootloader/consent.c)
target_compile_definitions(infrabot_firmware PRIVATE FW_NETBOOT_APP)
target_link_options(infrabot_firmware PRIVATE "-T${CMAKE_BINARY_DIR}/application.ld")
add_executable(netboot_loader bootloader/loader.c bootloader/staged_sha.c bootloader/slots.c bootloader/install.c bootloader/image.c bootloader/vendor/monocypher/monocypher.c bootloader/vendor/monocypher/monocypher-ed25519.c "${mbed}/library/sha256.c" ${common} boards/fire-h743-challenger-v2/main.c boards/fire-h743-challenger-v2/startup_stm32h743xx.s
  drivers/display/app_sdram.c app/app_startup.c app/app_persist.c drivers/net/ethernetif.c app/lwip_heap.c
  boards/fire-h743-challenger-v2/stm32h7xx_hal_msp.c boards/fire-h743-challenger-v2/syscalls.c boards/fire-h743-challenger-v2/sysmem.c
  third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sdram.c third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_fmc.c
  ${LWIP_CORE_SOURCES} ${ETH_COMPONENT_ROOT}/lan8742.c)
# Scope the optional PLL to the resident loader; the application has its own clock tree.
option(FW_BOOT_SCAN_PLL_192 "Temporarily use 192 MHz for ordinary boot verification instead of reset HSI (64 MHz)" OFF)
if(FW_BOOT_SCAN_PLL_192)
  target_sources(netboot_loader PRIVATE bootloader/scan_clock.c)
  target_compile_definitions(netboot_loader PRIVATE FW_BOOT_SCAN_PLL_192=1)
endif()
target_compile_options(netboot_loader PRIVATE -Os)
# Streaming SHA-256 over a full application image is on the cold boot path. Keep the
# larger speed-optimized code in sector 0 only, not in the tight app slots.
set_property(SOURCE "${mbed}/library/sha256.c" APPEND PROPERTY COMPILE_OPTIONS
  "$<$<STREQUAL:$<TARGET_PROPERTY:NAME>,netboot_loader>:-O3>")
target_compile_definitions(netboot_loader PRIVATE FW_LOADER_BUILD APP_LCD_SCRATCH_CACHE=0 HAL_SDRAM_MODULE_ENABLED)
target_link_libraries(netboot_loader PRIVATE STM32_Drivers stm32cubemx m)
target_link_options(netboot_loader PRIVATE "-T${CMAKE_BINARY_DIR}/loader.ld" "-Wl,-Map=netboot_loader.map")
foreach(t infrabot_firmware netboot_loader)
 target_include_directories(${t} PRIVATE bootloader bootloader/vendor/monocypher "${CMAKE_BINARY_DIR}/generated" "${mbed}/include"
    ${FREERTOS_ROOT}/include ${FREERTOS_ROOT}/portable/GCC/ARM_CM7/r0p1
    ${LWIP_ROOT}/src/include ${LWIP_ROOT}/system ${ETH_COMPONENT_ROOT})
endforeach()
set_property(TARGET netboot_loader APPEND PROPERTY LINK_DEPENDS "${CMAKE_BINARY_DIR}/loader.ld")
set_property(TARGET infrabot_firmware APPEND PROPERTY LINK_DEPENDS "${CMAKE_BINARY_DIR}/application.ld")
set_source_files_properties(${common} PROPERTIES COMPILE_OPTIONS "-Os")

# B is linked separately: do not relocate A's binary or change MCU option bytes.
string(REPLACE "0x08020400, LENGTH = 0xdfc00" "0x08100400, LENGTH = 0xdfc00" b_layout "${applayout}")
string(REPLACE "<= 0x08100000" "<= 0x081E0000" b_layout "${b_layout}")
file(WRITE "${CMAKE_BINARY_DIR}/application_b.ld" "${b_layout}")
get_target_property(app_sources infrabot_firmware SOURCES)
add_executable(infrabot_firmware_b ${app_sources})
foreach(prop INCLUDE_DIRECTORIES COMPILE_DEFINITIONS LINK_LIBRARIES)
  get_target_property(val infrabot_firmware ${prop})
  set_property(TARGET infrabot_firmware_b PROPERTY ${prop} "${val}")
endforeach()
add_dependencies(infrabot_firmware_b firmware_build_info)
target_link_options(infrabot_firmware_b PRIVATE "-T${CMAKE_BINARY_DIR}/application_b.ld" "-Wl,-Map=infrabot_firmware_b.map")
set_property(TARGET infrabot_firmware_b APPEND PROPERTY LINK_DEPENDS "${CMAKE_BINARY_DIR}/application_b.ld")

find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT FW_SIGNING_KEY_FILE)
  add_custom_target(netboot_images
    COMMAND ${CMAKE_COMMAND} -E echo "Set -DFW_SIGNING_KEY_FILE=/absolute/path/to/publisher.pem to build signed images."
    COMMAND ${CMAKE_COMMAND} -E false VERBATIM)
  return()
endif()
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/infrabot_firmware.ifw" "${CMAKE_BINARY_DIR}/factory-bank1.bin"
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_SOURCE_DIR}/tools/netboot_package.py"
      package --key "${FW_SIGNING_KEY_FILE}" --build "${CMAKE_BINARY_DIR}"
    DEPENDS netboot_loader infrabot_firmware infrabot_firmware_b tools/netboot_package.py tools/firmware_signing.py "${FW_SIGNING_KEY_FILE}"
    VERBATIM)
add_custom_target(netboot_images ALL DEPENDS "${CMAKE_BINARY_DIR}/infrabot_firmware.ifw" "${CMAKE_BINARY_DIR}/factory-bank1.bin")
