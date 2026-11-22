/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_log_task.h"

#include "app_config.h"
#include "app_ethernet.h"
#include "app_log.h"
#include "app_sdram.h"
#include "app_state.h"
#include "main.h"
#include "robot_controller.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* Text: 32 Ki lines of 256 bytes = 8 MiB, pruned to the newest 10 minutes.
 * 10 minutes fit up to ~54 lines/s; the status task prints ~2-6 lines/s.
 * Telemetry: 10 s at 1 kHz (+1 slot, the one being written) = 3.3 MiB.
 * Together ~11.3 MiB of the 30 MiB above the LCD/upgrade area. */
#define LOG_TEXT_LINES        32768U
#define LOG_TEXT_RETAIN_MS    (10U * 60U * 1000U)
#define LOG_TELEMETRY_RECORDS 10001U
#define LOG_TEXT_BYTES        (LOG_TEXT_LINES * (uint32_t)sizeof(AppLogTextRecord))
#define LOG_TELEMETRY_BYTES   (LOG_TELEMETRY_RECORDS * (uint32_t)sizeof(InfrabotTelemetryV3))
_Static_assert(APP_SDRAM_LOG_BASE - APP_SDRAM_BASE + LOG_TEXT_BYTES + LOG_TELEMETRY_BYTES <=
               APP_SDRAM_SIZE, "run log exceeds SDRAM");

/* printf -> log task.  Power of two; holds the boot banner printed before the
 * scheduler and the SDRAM start, and any burst between two 1 ms drains. */
#define LOG_FIFO_SIZE   16384U
#define LOG_DRAIN_BYTES 4096U   /* per tick, bounds the task's run time */
_Static_assert((LOG_FIFO_SIZE & (LOG_FIFO_SIZE - 1U)) == 0U, "FIFO size must be a power of two");

static uint8_t log_fifo[LOG_FIFO_SIZE];
static volatile uint32_t log_fifo_head, log_fifo_tail;  /* free-running counters */
static AppLogServer log_server;
static AppLogLine log_line;

void AppLogTask_Putc(uint8_t byte)
{
  const uint32_t head = log_fifo_head;
  if ((uint32_t)(head - log_fifo_tail) >= LOG_FIFO_SIZE)
  {
    log_server.lost_bytes = log_server.lost_bytes + 1U;
    return;
  }
  log_fifo[head & (LOG_FIFO_SIZE - 1U)] = byte;
  log_fifo_head = head + 1U;
}

static void AppLogTask_Drain(uint32_t now_ms)
{
  uint32_t tail = log_fifo_tail;
  const uint32_t head = log_fifo_head;
  uint32_t budget = LOG_DRAIN_BYTES;
  while (tail != head && budget-- != 0U)
  {
    if (AppLogLine_Feed(&log_line, log_fifo[tail & (LOG_FIFO_SIZE - 1U)]))
    {
      if (log_server.ready) AppLogText_Append(&log_server.text, now_ms, log_line.text, log_line.length);
      log_line.length = 0U;
    }
    ++tail;
  }
  log_fifo_tail = tail;
}

static void AppLogTask_Run(void *argument)
{
  (void)argument;
  if (AppSdram_Ready())
  {
    uint8_t *const base = (uint8_t *)APP_SDRAM_LOG_BASE;
    AppLogRing_Init(&log_server.text, base, sizeof(AppLogTextRecord), LOG_TEXT_LINES);
    AppLogRing_Init(&log_server.telemetry, base + LOG_TEXT_BYTES, sizeof(InfrabotTelemetryV3),
                    LOG_TELEMETRY_RECORDS);
    /* Distinguishes runs for the SDK; boot timing makes repeats unlikely, and
     * the SDK also notices the HAL tick going backwards. */
    log_server.boot_id = DWT->CYCCNT ^ (SysTick->VAL << 12) ^ HAL_GetTick() ^ 0x4C4F4721U;
    log_server.ready = 1U;
    printf("LOG: SDRAM run log at 0x%08lX: text %lu lines / %lu min, telemetry %lu ms; "
           "UDP port %u, heap_free=%lu\r\n",
           (unsigned long)APP_SDRAM_LOG_BASE, (unsigned long)LOG_TEXT_LINES,
           (unsigned long)(LOG_TEXT_RETAIN_MS / 60000U), (unsigned long)(LOG_TELEMETRY_RECORDS - 1U),
           (unsigned)INFRABOT_LOG_PORT, (unsigned long)xPortGetFreeHeapSize());
  }
  else
  {
    printf("LOG: SDRAM not running; MCU run log disabled\r\n");
  }

  const uint32_t cycles_per_us = (SystemCoreClock / 1000000U) != 0U ? SystemCoreClock / 1000000U : 1U;
  TickType_t wake = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&wake, 1U);
    const uint32_t start = DWT->CYCCNT;
    const uint32_t now_ms = HAL_GetTick();
    AppLogTask_Drain(now_ms);
    if (!log_server.ready) continue;
    AppLogText_Prune(&log_server.text, now_ms, LOG_TEXT_RETAIN_MS);
    AppLog_ExpireFreeze(&log_server, now_ms);
    if (!log_server.frozen && !log_server.latched)
    {
      InfrabotTelemetryV3 packet;
      AppEthernet_FillTelemetry(&packet);
      packet.head.sequence = log_server.telemetry.next;
      memcpy(AppLogRing_Begin(&log_server.telemetry), &packet, sizeof(packet));
      AppLogRing_Commit(&log_server.telemetry);
      /* A fault latches the ring right after its own record is stored, so the
       * 10 s before it and the fault tick stay until a download's RESUME.  While
       * a download holds the ring frozen the arm is disabled or idle and the
       * freeze already keeps what came before; a fault in that window latches on
       * the first tick after the freeze lifts. */
      const uint8_t status = (uint8_t)INFRABOT_STATUS_FROM_BYTE(packet.head.status);
      const uint8_t stop_reason = packet.safety.stop_reason;
      const uint8_t was_latched = log_server.latched;
      AppLog_FaultTick(&log_server, status, stop_reason);
      if (was_latched == 0U && log_server.latched != 0U)
        printf("LOG: fault (status=%u stop_reason=%u): telemetry ring latched until downloaded\r\n",
               (unsigned)status, (unsigned)stop_reason);
    }
    const uint32_t elapsed_us = (DWT->CYCCNT - start) / cycles_per_us;
    if (elapsed_us > log_server.writer_max_us) log_server.writer_max_us = elapsed_us;
  }
}

BaseType_t AppLogTask_Create(void)
{
  return xTaskCreate(AppLogTask_Run, "dm_log", DM_LOG_TASK_STACK, NULL, DM_LOG_TASK_PRIORITY, NULL);
}

size_t AppLogTask_Serve(const void *request, size_t size, uint8_t *reply)
{
  AppLogContext context;
  context.now_ms = HAL_GetTick();
  context.now_us = AppEthernet_GetTimeUs();
  /* Disabled, idle, stopped or faulted arms may download; enabling and every motion
   * mode refuse, so no log traffic shares the link or this task with motion.  The
   * status is recomputed on each call and FAULT outranks the control mode in it,
   * so a fault that happens in RT mode is served too. */
  context.busy = (uint8_t)!AppLog_DownloadAllowed(RobotController_OperationalStatus());
  return AppLog_Serve(&log_server, request, size, &context, reply);
}
