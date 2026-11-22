/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "FreeRTOS.h"

/* Keep task stacks and queue allocations out of DTCM.  Ethernet/LwIP already
 * uses D2 SRAM; AXI SRAM is a good general-purpose pool for RTOS objects. */
uint8_t ucHeap[configTOTAL_HEAP_SIZE]
  __attribute__((section(".freertos_heap"), aligned(8)));
