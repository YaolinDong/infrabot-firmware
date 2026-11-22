# Firmware dependencies

The default build uses the pinned STM32CubeH7 V1.13.0 sources below.

| Component | Version | License | Source in Cube package |
|---|---|---|---|
| [FreeRTOS](STM32CubeH7/Middlewares/Third_Party/FreeRTOS/README.md) | 10.6.2 | MIT | `Middlewares/Third_Party/FreeRTOS` |
| [LwIP](STM32CubeH7/Middlewares/Third_Party/LwIP/README.md) | 2.2.1 | BSD-3-Clause (see COPYING and individual file notices) | `Middlewares/Third_Party/LwIP` |
| [mbedTLS](STM32CubeH7/Middlewares/Third_Party/mbedTLS/README.md) | 2.16.2 | Apache-2.0 (see LICENSE and individual file notices) | `Middlewares/Third_Party/mbedTLS` |
| [LAN8742](STM32CubeH7/Drivers/BSP/Components/lan8742/README.md) | 1.0.4 | BSD-3-Clause (see LICENSE.txt) | `Drivers/BSP/Components/lan8742` |
| [STM32H7xx HAL/LL](STM32H7xx_HAL_Driver/LICENSE.txt) | 1.11.6 | BSD-3-Clause | Drivers/STM32H7xx_HAL_Driver |
| [CMSIS Core (M)](CMSIS/LICENSE.txt) | 5.6 | Apache-2.0 | Drivers/CMSIS/Include |
| [STM32H7 device support](CMSIS/Device/ST/STM32H7xx/LICENSE.txt) | STM32CubeH7 V1.13.0 | Apache-2.0 | Drivers/CMSIS/Device/ST/STM32H7xx |

The Ethernet HAL differs from the upstream header. [Patch record](PATCHES.md) · [Distribution notices](../THIRD_PARTY.md)
