/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include <string.h>
#include <stddef.h>
typedef enum {HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,HAL_TIMEOUT=3} HAL_StatusTypeDef;
typedef struct { uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator, BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker; } FDCAN_TxHeaderTypeDef;
typedef struct { int dummy; } FDCAN_HandleTypeDef;
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_ON 1
#define FDCAN_BRS_OFF 0
#define FDCAN_FD_CAN 1
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_DLC_BYTES_8 8
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, const FDCAN_TxHeaderTypeDef *hd, uint8_t *d);
