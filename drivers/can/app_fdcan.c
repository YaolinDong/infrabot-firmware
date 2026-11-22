/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "bus_state.h"
#include "app_fdcan.h"

#include "app_state.h"
#include "dm_motor.h"
#include "app_damiao.h"

static uint8_t AppFdcan_DlcToLength(uint32_t data_length)
{
  switch (data_length)
  {
    case FDCAN_DLC_BYTES_0:  return 0U;
    case FDCAN_DLC_BYTES_1:  return 1U;
    case FDCAN_DLC_BYTES_2:  return 2U;
    case FDCAN_DLC_BYTES_3:  return 3U;
    case FDCAN_DLC_BYTES_4:  return 4U;
    case FDCAN_DLC_BYTES_5:  return 5U;
    case FDCAN_DLC_BYTES_6:  return 6U;
    case FDCAN_DLC_BYTES_7:  return 7U;
    case FDCAN_DLC_BYTES_8:  return 8U;
    case FDCAN_DLC_BYTES_12:
    case FDCAN_DLC_BYTES_16:
    case FDCAN_DLC_BYTES_20:
    case FDCAN_DLC_BYTES_24:
    case FDCAN_DLC_BYTES_32:
    case FDCAN_DLC_BYTES_48:
    case FDCAN_DLC_BYTES_64:
    default:                 return 8U;
  }
}

void AppFdcan_StartForDmMotor(uint8_t use_fd)
{
  FDCAN_FilterTypeDef filter = {0};

  if (HAL_FDCAN_DeInit(&hfdcan2) != HAL_OK) Error_Handler();

  hfdcan2.Init.FrameFormat = (use_fd != 0U) ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = ENABLE;
  hfdcan2.Init.NominalPrescaler = DM_CAN_NOMINAL_PRESCALER;
  hfdcan2.Init.NominalSyncJumpWidth = DM_CAN_NOMINAL_SJW;
  hfdcan2.Init.NominalTimeSeg1 = DM_CAN_NOMINAL_SEG1;
  hfdcan2.Init.NominalTimeSeg2 = DM_CAN_NOMINAL_SEG2;
  hfdcan2.Init.DataPrescaler = DM_CAN_DATA_PRESCALER;
  hfdcan2.Init.DataSyncJumpWidth = DM_CAN_DATA_SJW;
  hfdcan2.Init.DataTimeSeg1 = DM_CAN_DATA_SEG1;
  hfdcan2.Init.DataTimeSeg2 = DM_CAN_DATA_SEG2;
  hfdcan2.Init.MessageRAMOffset = 0U;
  hfdcan2.Init.StdFiltersNbr = 1U;
  hfdcan2.Init.ExtFiltersNbr = 0U;
  hfdcan2.Init.RxFifo0ElmtsNbr = 8U;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0U;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0U;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0U;
  hfdcan2.Init.TxBuffersNbr = 0U;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 32U;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) Error_Handler();

#if DM_CAN_TDC_ENABLE
  if (use_fd != 0U)
  {
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2,
                                            DM_CAN_TDC_OFFSET,
                                            DM_CAN_TDC_FILTER) != HAL_OK)
      Error_Handler();
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2) != HAL_OK) Error_Handler();
  }
#endif

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0U;
  filter.FilterID2 = 0U;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK) Error_Handler();
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
    Error_Handler();
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) Error_Handler();
  if (HAL_FDCAN_ActivateNotification(&hfdcan2,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_BUS_OFF |
                                     FDCAN_IT_ARB_PROTOCOL_ERROR |
                                     FDCAN_IT_DATA_PROTOCOL_ERROR |
                                     FDCAN_IT_ERROR_PASSIVE |
                                     FDCAN_IT_ERROR_WARNING,
                                     0U) != HAL_OK)
    Error_Handler();
}

uint8_t AppFdcan_BusFault(void)
{
  const uint32_t ecr = hfdcan2.Instance->ECR;
  const uint32_t tec = (ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos;

  /* Bus-off sets CCCR.INIT; RP is receive error passive. */
  return ((hfdcan2.Instance->CCCR & FDCAN_CCCR_INIT) != 0U) ||
         ((ecr & FDCAN_ECR_RP) != 0U) || (tec >= 128U);
}

/* Error warning / error passive / bus-off transitions. */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                    uint32_t ErrorStatusITs)
{
  if (hfdcan == &hfdcan2)
  {
    dm_bus_stats.can_error_events++;
    dm_bus_stats.motor_error_status = ErrorStatusITs;
  }
}

/* Protocol errors.  HAL ORs them into ErrorCode and calls this on every
 * later interrupt while ErrorCode is non-zero, so take the code and clear
 * it: one event per error, not per received frame. */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan == &hfdcan2 && hfdcan->ErrorCode != HAL_FDCAN_ERROR_NONE)
  {
    dm_bus_stats.can_error_events++;
    dm_bus_stats.motor_error_code = hfdcan->ErrorCode;
    hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
  }
}

static void AppFdcan_ProcessRxFrame(const DM_CanRxFrame_t *frame)
{
  if (frame == NULL) return;
  dm_bus_stats.rx_frame_count++;

  uint8_t matched = 0U;
  const uint8_t register_frame = AppDamiao_RegisterRx(frame);
  if (!register_frame && (dm_bus_stats.feedback_enabled != 0U) && (frame->length == 8U))
  {
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    {
      matched = DM_Motor_ProcessFeedback(&dm_motors[i], frame->id, frame->data, 8U);
      if (matched != 0U) break;
    }
    if (matched != 0U) dm_bus_stats.feedback_match_count++;
  }

  if (matched == 0U)
  {
    uint32_t head = dm_bus_stats.rx_trace_head;
    if ((head - dm_bus_stats.rx_trace_tail) >= DM_RX_TRACE_DEPTH)
    {
      dm_bus_stats.rx_trace_dropped++;
    }
    else
    {
      dm_bus_stats.rx_trace[head % DM_RX_TRACE_DEPTH] = *frame;
      dm_bus_stats.rx_trace_head = head + 1U;
    }
  }
  else if (frame->fd_format != 0U)
  {
    dm_bus_stats.feedback_fd_count++;
  }
  else
  {
    dm_bus_stats.feedback_classic_count++;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  (void)RxFifo0ITs;
  if (hfdcan != &hfdcan2) return;

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    FDCAN_RxHeaderTypeDef header = {0};
    DM_CanRxFrame_t frame = {0};
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK)
      break;

    frame.id = (uint16_t)header.Identifier;
    frame.length = AppFdcan_DlcToLength(header.DataLength);
    frame.fd_format = (header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U;
    frame.brs = (header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;
    AppFdcan_ProcessRxFrame(&frame);
  }
}
