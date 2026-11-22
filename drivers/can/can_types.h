/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>

typedef struct
{
  uint16_t id;
  uint8_t length;
  uint8_t fd_format;
  uint8_t brs;
  uint8_t data[8];
} DM_CanRxFrame_t;
