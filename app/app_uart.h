/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_UART_H
#define APP_UART_H

/* Non-blocking debug UART transport. printf only appends to the ring; the
 * USART interrupt drains bounded contiguous segments. */
void AppUart_TxKick(void);

/* Bounded watchdog/restart service, called from the control task. */
void AppUart_Service(void);

#endif /* APP_UART_H */
