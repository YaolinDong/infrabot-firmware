# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms
	USE_PWR_LDO_SUPPLY
	USE_HAL_DRIVER
	STM32H743xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    # The application owns the application ethernetif/lwipopts headers.  Put it first so
    # CubeMX's generated headers cannot shadow the no-sys integration.
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/control
    ${CMAKE_SOURCE_DIR}/drivers/damiao
    ${CMAKE_SOURCE_DIR}/drivers/can
    ${CMAKE_SOURCE_DIR}/drivers/net
    ${CMAKE_SOURCE_DIR}/drivers/display
    ${CMAKE_SOURCE_DIR}/ui
    ${CMAKE_SOURCE_DIR}/protocol
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2
    ${LWIP_ROOT}/src/include
    ${LWIP_ROOT}/system
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Inc/Legacy
    ${ETH_COMPONENT_ROOT}
    ${LWIP_ROOT}/src/include/netif/ppp
    ${CMAKE_SOURCE_DIR}/third_party/CMSIS/Device/ST/STM32H7xx/Include
    ${LWIP_ROOT}/src/include/lwip
    ${LWIP_ROOT}/src/include/lwip/apps
    ${LWIP_ROOT}/src/include/lwip/priv
    ${LWIP_ROOT}/src/include/lwip/prot
    ${LWIP_ROOT}/src/include/netif
    ${LWIP_ROOT}/src/include/compat/posix
    ${LWIP_ROOT}/src/include/compat/posix/arpa
    ${LWIP_ROOT}/src/include/compat/posix/net
    ${LWIP_ROOT}/src/include/compat/posix/sys
    ${LWIP_ROOT}/src/include/compat/stdc
    ${LWIP_ROOT}/system/arch
    ${CMAKE_SOURCE_DIR}/third_party/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    # drivers/net owns ethernetif.c and app_ethernet.c.  Exclude CubeMX's
    # standalone ethernet startup to avoid a second ETH handle/netif.
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/main.c
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/stm32h7xx_it.c
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/stm32h7xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/sysmem.c
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/syscalls.c
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/startup_stm32h743xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/boards/fire-h743-challenger-v2/system_stm32h7xx.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth_ex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ${CMAKE_SOURCE_DIR}/third_party/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms}
    PLATFORM_IMPL="platform_impl.h" PLATFORM_DRIVE_IMPL="platform_drive_impl.h")

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
