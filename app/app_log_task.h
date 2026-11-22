/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LOG_TASK_H
#define APP_LOG_TASK_H

/* MCU run log (see app_log.h): the SDRAM rings, the 1 kHz writer task and the
 * UDP request handler.  Never stops or slows the arm: no code here runs in the
 * control task, and a failure only disables the log. */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"

/* Called by __io_putchar with interrupts disabled: one byte into a DTCM FIFO
 * that the log task drains.  A full FIFO drops the byte (counted). */
void AppLogTask_Putc(uint8_t byte);

/* Creates the writer task (DM_LOG_TASK_PRIORITY).  Before the scheduler. */
BaseType_t AppLogTask_Create(void);

/* Ethernet task: answers one DMLG datagram; returns the DMLR length (0 = no
 * reply).  reply holds sizeof(InfrabotLogReplyHeader) + INFRABOT_LOG_DATA_MAX. */
size_t AppLogTask_Serve(const void *request, size_t size, uint8_t *reply);

#endif /* APP_LOG_TASK_H */
