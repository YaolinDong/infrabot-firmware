# Local differences

In STM32H7xx HAL 1.11.6, `Inc/stm32h7xx_hal_eth.h` defines `ETH_MDIO_BUS_TIMEOUT` as 1000U unless overridden. Commit 4188e5a changed the default in our copy to 20U. This bounds an unanswered PHY management transaction during network probe. Retain the value when updating the HAL and check boot with the link down.
