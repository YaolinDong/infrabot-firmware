/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "lwipopts.h"

/* LwIP's private heap is in D2 SRAM, alongside the Ethernet DMA objects.
 * The linker places it after the DMA pool; using a symbol instead of a hard
 * coded address keeps it safe if the descriptor pool size changes. */
unsigned char lwip_heap[LWIP_HEAP_REGION_SIZE]
  __attribute__((section(".lwip_heap"), aligned(32)));
