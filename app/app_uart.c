/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "uart_tx_state.h"
#include "app_uart.h"

#include "app_log_task.h"
#include "app_runtime.h"
#include "app_state.h"

#define APP_UART_TX_WATCHDOG_MS 100U

static uint32_t AppUart_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return primask;
}

static void AppUart_ExitCritical(uint32_t primask)
{
  __DMB();
  __set_PRIMASK(primask);
}

static void AppUart_ResetHalTx(void)
{
  ATOMIC_CLEAR_BIT(huart1.Instance->CR1, USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE);
  ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_TXFTIE);
  huart1.TxISR = NULL;
  huart1.gState = HAL_UART_STATE_READY;
}

static void AppUart_RecordHalError(void)
{
  uart_tx.hal_errors++;
  uart_tx.last_error = (uint32_t)huart1.ErrorCode;
  uart_tx.last_state = (uint32_t)huart1.gState;
}

void AppUart_TxKick(void)
{
  uint16_t length = 0U;
  uint32_t tail = 0U;
  uint32_t primask = AppUart_EnterCritical();

  if ((uart_tx.busy == 0U) && (uart_tx.tail != uart_tx.head))
  {
    tail = uart_tx.tail;
    uint32_t head = uart_tx.head;
    length = (head > tail) ? (uint16_t)(head - tail)
                           : (uint16_t)(DM_UART_TX_RING_SIZE - tail);
    /* Keep interrupt service bounded while preserving the ring contents. */
    if (length > 256U) length = 256U;
    uart_tx.active_len = length;
    uart_tx.busy = 1U;
    uart_tx.start_tick = HAL_GetTick();
    /* Keep reservation and HAL state transition atomic with respect to a
       pending TC interrupt.  The UART IRQ runs after interrupts are restored. */
    if (HAL_UART_Transmit_IT(&huart1, &uart_tx.ring[tail], length) != HAL_OK)
    {
      AppUart_RecordHalError();
      AppUart_ResetHalTx();
      uart_tx.busy = 0U;
      uart_tx.active_len = 0U;
      uart_tx.start_tick = 0U;
    }
  }
  AppUart_ExitCritical(primask);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;

  uint32_t primask = AppUart_EnterCritical();
  uart_tx.tail += uart_tx.active_len;
  while (uart_tx.tail >= DM_UART_TX_RING_SIZE)
    uart_tx.tail -= DM_UART_TX_RING_SIZE;
  uart_tx.active_len = 0U;
  uart_tx.busy = 0U;
  uart_tx.start_tick = 0U;
  AppUart_ExitCritical(primask);
  AppUart_TxKick();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;
  uint32_t primask = AppUart_EnterCritical();
  AppUart_RecordHalError();
  AppUart_ResetHalTx();
  uart_tx.busy = 0U;
  uart_tx.active_len = 0U;
  uart_tx.start_tick = 0U;
  AppUart_ExitCritical(primask);
  AppUart_TxKick();
}

void AppUart_Service(void)
{
  uint32_t primask = AppUart_EnterCritical();
  if ((uart_tx.busy != 0U) &&
      ((HAL_GetTick() - uart_tx.start_tick) > APP_UART_TX_WATCHDOG_MS))
  {
    AppUart_RecordHalError();
    AppUart_ResetHalTx();
    uart_tx.busy = 0U;
    uart_tx.active_len = 0U;
    uart_tx.start_tick = 0U;
    uart_tx.recoveries++;
  }
  AppUart_ExitCritical(primask);

  /* Also starts a pending segment when a previous HAL call returned busy. */
  AppUart_TxKick();
}

int __io_putchar(int ch)
{
  uint32_t next;
  uint32_t primask = AppUart_EnterCritical();

  /* The run log keeps every byte, also those the UART ring has to drop. */
  AppLogTask_Putc((uint8_t)ch);
  next = uart_tx.head + 1U;
  if (next >= DM_UART_TX_RING_SIZE) next = 0U;
  if (next == uart_tx.tail)
  {
    uart_tx.dropped++;
    AppUart_ExitCritical(primask);
    return ch;
  }
  uart_tx.ring[uart_tx.head] = (uint8_t)ch;
  uart_tx.head = next;
  AppUart_ExitCritical(primask);

  AppUart_TxKick();
  return ch;
}
