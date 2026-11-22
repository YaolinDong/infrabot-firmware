/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LOG_H
#define APP_LOG_H

/* MCU-side run log kept in external SDRAM (app_log_task.c), read by the SDK
 * over UDP (INFRABOT_LOG_PORT, protocol.h "MCU run log"):
 *   - text: every byte printf sends to USART1, as lines stamped with the
 *     HAL millisecond tick; lines older than 10 minutes are dropped;
 *   - telemetry: one DMTL v3 snapshot per control tick; the newest 10 s.
 * Lost on power-off.  The SDRAM is not refreshed across a reset, so both
 * rings start empty on every boot.
 *
 * This header is the ring and request logic only: no HAL, no RTOS, so the
 * host tests run exactly what runs on the MCU.
 *
 * Concurrency on the MCU: one writer (the log task) and one reader (the
 * Ethernet task, higher priority).  The reader can interrupt the writer,
 * never the reverse, so the ring indices are constant during one request.
 * The writer may be interrupted while filling the slot of index `next`; that
 * slot is never readable (the readable range ends at next - 1 and starts at
 * next - (count - 1) at the earliest). */

#include <stddef.h>
#include <stdint.h>

#include "infrabot/protocol.h"

/* ---- Fixed-size record ring -----------------------------------------------------
 * Record n lives in slot n % count; n is a wrapping 32-bit index. */
typedef struct {
  uint8_t *base;
  uint32_t record_size, count;
  volatile uint32_t next;   /* index of the record being written */
  volatile uint32_t kept;   /* readable records: [next - kept, next), kept <= count - 1 */
} AppLogRing;

void AppLogRing_Init(AppLogRing *ring, void *base, uint32_t record_size, uint32_t count);
void *AppLogRing_Slot(const AppLogRing *ring, uint32_t index);
void *AppLogRing_Begin(AppLogRing *ring);   /* the slot of `next` */
void AppLogRing_Commit(AppLogRing *ring);   /* publishes it */
uint32_t AppLogRing_First(const AppLogRing *ring);
void AppLogRing_DropOldest(AppLogRing *ring);

/* ---- Text records ----------------------------------------------------------------- */
#define APP_LOG_TEXT_LINE_MAX 246U
typedef struct {
  uint32_t seq;      /* == ring index once committed */
  uint32_t time_ms;  /* HAL tick when the line was stored */
  uint16_t length;
  char text[APP_LOG_TEXT_LINE_MAX];
} AppLogTextRecord;
_Static_assert(sizeof(AppLogTextRecord) == 256U, "text record size");

void AppLogText_Append(AppLogRing *ring, uint32_t time_ms, const char *text, uint32_t length);
/* Drops lines older than retain_ms (wrapping tick arithmetic). */
void AppLogText_Prune(AppLogRing *ring, uint32_t now_ms, uint32_t retain_ms);

/* ---- printf bytes -> lines --------------------------------------------------------
 * '\r' is dropped; '\n' ends a line; a line longer than APP_LOG_TEXT_LINE_MAX
 * is cut there and continues as the next line. */
typedef struct {
  char text[APP_LOG_TEXT_LINE_MAX];
  uint32_t length;
} AppLogLine;

/* Returns 1 when line holds a complete line; the caller stores it and sets
 * length = 0. */
int AppLogLine_Feed(AppLogLine *line, uint8_t byte);

/* ---- UDP request server ------------------------------------------------------------ */
#define APP_LOG_REPLIES_PER_MS    2U      /* further requests in the same ms get no reply */
#define APP_LOG_FREEZE_TIMEOUT_MS 5000U   /* telemetry resumes after this long without a request */

typedef struct {
  AppLogRing text, telemetry;
  uint32_t boot_id;
  uint8_t ready;                     /* SDRAM running and rings initialised */
  volatile uint8_t frozen;           /* telemetry recording paused for a download (expires) */
  volatile uint8_t latched;          /* paused by a fault: kept until a RESUME, never expires */
  uint8_t fault_prev;                /* writer only: a fault was present on the previous tick */
  volatile uint32_t last_request_ms;
  volatile uint32_t lost_bytes;      /* printf bytes the log could not take */
  volatile uint32_t writer_max_us;   /* longest log-task tick since the last STATUS */
  uint32_t serve_ms, serve_count;
} AppLogServer;

typedef struct {
  uint32_t now_ms;   /* HAL tick: the text time_ms clock */
  uint64_t now_us;   /* DWT clock: DMTL mcu_timestamp_us */
  uint8_t busy;      /* arm not disabled/idle: refuse (AppLog_DownloadAllowed) */
} AppLogContext;

/* Answers one datagram.  reply must hold sizeof(InfrabotLogReplyHeader) +
 * INFRABOT_LOG_DATA_MAX bytes.  Returns the reply length; 0 = no reply. */
size_t AppLog_Serve(AppLogServer *server, const void *request, size_t request_size,
                    const AppLogContext *context, uint8_t *reply);

/* Downloads are served while the arm is disabled, holding in IDLE, or stopped:
 * controlled stop (STOPPING), FAULT, ESTOP.  A stop or fault is when the log is
 * wanted most.  Enabling, recovery and every motion mode (RT/NRT, impedance,
 * admittance, gravity) refuse, so a download never competes with motion traffic. */
static inline uint8_t AppLog_DownloadAllowed(uint8_t status)
{
  return (uint8_t)(status == INFRABOT_STATUS_DISABLED || status == INFRABOT_STATUS_IDLE_HOLD ||
                   status == INFRABOT_STATUS_STOPPING || status == INFRABOT_STATUS_FAULT ||
                   status == INFRABOT_STATUS_ESTOP);
}

/* A fault whose lead-up the telemetry ring must keep: controller state FAULT or
 * ESTOP, or a latched stop reason that is not the operator's own STOP. */
uint8_t AppLog_IsFault(uint8_t status, uint8_t stop_reason);

/* Writer, every tick, after it stored this tick's record: on the first tick a
 * fault appears, latches the telemetry ring so the 10 s before it stay until a
 * RESUME.  A fault that is still present after the RESUME does not latch again. */
void AppLog_FaultTick(AppLogServer *server, uint8_t status, uint8_t stop_reason);

/* Called by the writer every tick: lifts a freeze nobody finished. */
void AppLog_ExpireFreeze(AppLogServer *server, uint32_t now_ms);

#endif /* APP_LOG_H */
