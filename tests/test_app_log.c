/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* MCU run log (app_log.c): ring wrap, time pruning, line splitting, the DMLG
 * request server (clamping, paging, rate limit, busy/unavailable, freeze),
 * and a reader that runs while the writer is mid-record. */
#include "app_log.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t reply[sizeof(InfrabotLogReplyHeader) + INFRABOT_LOG_DATA_MAX];

static InfrabotLogReplyHeader ask(AppLogServer *server, uint8_t op, uint32_t cursor, uint32_t max_bytes,
                                  uint32_t now_ms, uint8_t busy, size_t *length)
{
  static uint32_t id;
  InfrabotLogRequest request = {INFRABOT_MAGIC_LOG_REQUEST, INFRABOT_LOG_V1, op, 0U, ++id, cursor, max_bytes};
  AppLogContext context = {now_ms, (uint64_t)now_ms * 1000U + 7U, busy};
  *length = AppLog_Serve(server, &request, sizeof(request), &context, reply);
  InfrabotLogReplyHeader header;
  memset(&header, 0, sizeof(header));
  if (*length) memcpy(&header, reply, sizeof(header));
  if (*length) assert(header.request_id == id && header.magic == INFRABOT_MAGIC_LOG_REPLY);
  if (*length) assert(*length == sizeof(header) + header.data_bytes);
  return header;
}

static void feed(AppLogLine *line, AppLogRing *ring, uint32_t now, const char *text)
{
  for (const char *c = text; *c; ++c)
    if (AppLogLine_Feed(line, (uint8_t)*c)) { AppLogText_Append(ring, now, line->text, line->length); line->length = 0U; }
}

static void test_lines(void)
{
  static AppLogTextRecord slots[16];
  AppLogRing ring;
  AppLogRing_Init(&ring, slots, sizeof(AppLogTextRecord), 16U);
  AppLogLine line = {{0}, 0U};
  feed(&line, &ring, 5U, "STAT IDLE miss=0\r\nGRAV us=1");
  assert(ring.next == 1U && ring.kept == 1U);
  feed(&line, &ring, 6U, "78\r\n\r\n");
  assert(ring.next == 3U);
  assert(slots[0].length == 16U && memcmp(slots[0].text, "STAT IDLE miss=0", 16U) == 0);
  /* A line split across two printf bursts is stored once, whole. */
  assert(slots[1].length == 11U && memcmp(slots[1].text, "GRAV us=178", 11U) == 0);
  assert(slots[1].seq == 1U && slots[1].time_ms == 6U);
  assert(slots[2].length == 0U);  /* empty line kept as an empty record */
  /* A line longer than the record is cut and continues. */
  char big[APP_LOG_TEXT_LINE_MAX + 20U + 2U];
  memset(big, 'x', APP_LOG_TEXT_LINE_MAX + 20U);
  big[APP_LOG_TEXT_LINE_MAX + 20U] = '\n';
  big[APP_LOG_TEXT_LINE_MAX + 21U] = '\0';
  feed(&line, &ring, 7U, big);
  assert(ring.next == 5U);
  assert(slots[3].length == APP_LOG_TEXT_LINE_MAX && slots[4].length == 20U);
  assert(slots[3].seq == 3U && slots[4].seq == 4U && slots[4].time_ms == 7U);
}

static void test_ring_and_prune(void)
{
  static AppLogTextRecord slots[8];
  AppLogRing ring;
  AppLogRing_Init(&ring, slots, sizeof(AppLogTextRecord), 8U);
  for (uint32_t i = 0; i < 20U; ++i) AppLogText_Append(&ring, 1000U * i, "a", 1U);
  /* count - 1 readable: the slot of `next` is the one the writer fills next. */
  assert(ring.next == 20U && ring.kept == 7U && AppLogRing_First(&ring) == 13U);
  for (uint32_t i = AppLogRing_First(&ring); i != ring.next; ++i)
    assert(((AppLogTextRecord *)AppLogRing_Slot(&ring, i))->seq == i);
  AppLogText_Prune(&ring, 19000U, 3000U);   /* keep time >= 16000 */
  assert(AppLogRing_First(&ring) == 16U && ring.kept == 4U);
  AppLogText_Prune(&ring, 0xFFFFFFFFU, 3000U);  /* far in the future: everything old */
  assert(ring.kept == 0U && AppLogRing_First(&ring) == ring.next);
  /* Tick wrap: records stamped just before 2^32 survive a "now" just after. */
  AppLogText_Append(&ring, 0xFFFFFF00U, "w", 1U);
  AppLogText_Prune(&ring, 0x00000100U, 3000U);
  assert(ring.kept == 1U);
  /* Index wrap at 2^32. */
  ring.next = 0xFFFFFFFEU; ring.kept = 0U;
  for (int i = 0; i < 4; ++i) AppLogText_Append(&ring, 1U, "z", 1U);
  assert(ring.next == 2U && ring.kept == 4U && AppLogRing_First(&ring) == 0xFFFFFFFEU);
}

static void test_server(void)
{
  static AppLogTextRecord text_slots[64];
  static InfrabotTelemetryV3 telemetry_slots[11];
  AppLogServer server;
  memset(&server, 0, sizeof(server));
  size_t length = 0U;

  /* Not ready: UNAVAILABLE, never data. */
  InfrabotLogReplyHeader h = ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 1U, 0U, &length);
  assert(length == sizeof(h) && h.result == INFRABOT_LOG_UNAVAILABLE);

  AppLogRing_Init(&server.text, text_slots, sizeof(AppLogTextRecord), 64U);
  AppLogRing_Init(&server.telemetry, telemetry_slots, sizeof(InfrabotTelemetryV3), 11U);
  server.boot_id = 0xB007U;
  server.ready = 1U;

  /* Malformed requests get nothing. */
  InfrabotLogRequest bad = {0x12345678U, INFRABOT_LOG_V1, 0U, 0U, 1U, 0U, 0U};
  AppLogContext context = {2U, 0U, 0U};
  assert(AppLog_Serve(&server, &bad, sizeof(bad), &context, reply) == 0U);
  assert(AppLog_Serve(&server, &bad, sizeof(bad) - 1U, &context, reply) == 0U);
  /* Unknown op / version: INVALID. */
  h = ask(&server, 9U, 0U, 0U, 3U, 0U, &length);
  assert(h.result == INFRABOT_LOG_INVALID);

  /* Busy: a PC real-time stream owns the link. */
  h = ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 4U, 1U, &length);
  assert(h.result == INFRABOT_LOG_BUSY && h.data_bytes == 0U);

  /* 40 lines of 1..40 characters. */
  char text[64];
  for (uint32_t i = 0; i < 40U; ++i)
  {
    for (uint32_t k = 0; k <= i; ++k) text[k] = (char)('A' + (i + k) % 26U);
    AppLogText_Append(&server.text, 100U + i, text, i + 1U);
  }
  server.writer_max_us = 37U;
  h = ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 5U, 0U, &length);
  assert(h.result == INFRABOT_LOG_OK && h.boot_id == 0xB007U && h.writer_max_us == 37U);
  assert(h.text_first == 0U && h.text_next == 40U && h.now_ms == 5U && h.now_us == 5007U);
  h = ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 6U, 0U, &length);
  assert(h.writer_max_us == 0U);  /* reset by the previous STATUS */

  /* Page through with a small page: every line exactly once, in order. */
  uint32_t cursor = 0U, seen = 0U;
  for (uint32_t now = 10U; cursor != 40U; now += 1U)
  {
    h = ask(&server, INFRABOT_LOG_OP_TEXT, cursor, 120U, now, 0U, &length);
    assert(h.result == INFRABOT_LOG_OK && h.cursor == cursor && h.data_bytes <= 120U);
    assert(h.record_count > 0U);
    const uint8_t *p = reply + sizeof(h);
    for (uint8_t r = 0; r < h.record_count; ++r)
    {
      uint32_t seq, time_ms; uint16_t len;
      memcpy(&seq, p, 4); memcpy(&time_ms, p + 4, 4); memcpy(&len, p + 8, 2);
      assert(seq == seen && time_ms == 100U + seen && len == seen + 1U);
      for (uint32_t k = 0; k < len; ++k) assert(p[10 + k] == (uint8_t)('A' + (seen + k) % 26U));
      p += (10U + len + 3U) & ~3U;
      ++seen;
    }
    assert(p == reply + sizeof(h) + h.data_bytes);
    cursor = h.next_cursor;
  }
  assert(seen == 40U);
  /* At the end: an empty page, cursor unchanged. */
  h = ask(&server, INFRABOT_LOG_OP_TEXT, 40U, 0U, 60U, 0U, &length);
  assert(h.record_count == 0U && h.next_cursor == 40U);
  /* A cursor that was already dropped (or garbage) restarts at the oldest. */
  AppLogText_Prune(&server.text, 100U + 30U + 1000U, 1000U);
  assert(AppLogRing_First(&server.text) == 30U);
  h = ask(&server, INFRABOT_LOG_OP_TEXT, 3U, 0U, 61U, 0U, &length);
  assert(h.cursor == 30U && h.text_first == 30U);
  h = ask(&server, INFRABOT_LOG_OP_TEXT, 0x80000000U, 0U, 62U, 0U, &length);
  assert(h.cursor == 30U);

  /* Rate limit: at most APP_LOG_REPLIES_PER_MS replies per millisecond. */
  for (uint32_t i = 0; i < APP_LOG_REPLIES_PER_MS; ++i)
  {
    h = ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 70U, 0U, &length);
    assert(length != 0U);
  }
  (void)ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 70U, 0U, &length);
  assert(length == 0U);
  (void)ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 71U, 0U, &length);
  assert(length != 0U);

  /* Telemetry: 4 records per full page, sequence = index. */
  for (uint32_t i = 0; i < 25U; ++i)
  {
    InfrabotTelemetryV3 *slot = (InfrabotTelemetryV3 *)AppLogRing_Begin(&server.telemetry);
    memset(slot, 0, sizeof(*slot));
    slot->head.magic = INFRABOT_MAGIC_TELEMETRY;
    slot->head.sequence = server.telemetry.next;
    slot->head.mcu_timestamp_us = 1000U * i;
    AppLogRing_Commit(&server.telemetry);
  }
  h = ask(&server, INFRABOT_LOG_OP_TELEMETRY, 0U, 0U, 80U, 0U, &length);
  assert(h.telemetry_first == 15U && h.telemetry_next == 25U && h.cursor == 15U);
  assert(h.record_count == 4U && h.data_bytes == 4U * sizeof(InfrabotTelemetryV3) && h.next_cursor == 19U);
  for (uint8_t r = 0; r < h.record_count; ++r)
  {
    InfrabotTelemetryV3 record;
    memcpy(&record, reply + sizeof(h) + r * sizeof(record), sizeof(record));
    assert(record.head.sequence == 15U + r && record.head.mcu_timestamp_us == 1000U * (15U + r));
  }
  /* A page too small for one record returns none (no partial records). */
  h = ask(&server, INFRABOT_LOG_OP_TELEMETRY, 19U, sizeof(InfrabotTelemetryV3) - 1U, 81U, 0U, &length);
  assert(h.record_count == 0U && h.next_cursor == 19U);

  /* Freeze / resume / expiry. */
  h = ask(&server, INFRABOT_LOG_OP_FREEZE, 0U, 0U, 90U, 0U, &length);
  assert(h.frozen == 1U && server.frozen == 1U);
  AppLog_ExpireFreeze(&server, 90U + APP_LOG_FREEZE_TIMEOUT_MS);
  assert(server.frozen == 1U);
  AppLog_ExpireFreeze(&server, 91U + APP_LOG_FREEZE_TIMEOUT_MS);
  assert(server.frozen == 0U);
  (void)ask(&server, INFRABOT_LOG_OP_FREEZE, 0U, 0U, 100U, 0U, &length);
  h = ask(&server, INFRABOT_LOG_OP_RESUME, 0U, 0U, 101U, 0U, &length);
  assert(h.frozen == 0U && server.frozen == 0U);
}

/* The Ethernet task (reader) preempts the log task (writer) at any point of
 * an append.  Simulate every interleaving: the reader must never return the
 * slot being written, and every record it returns must be whole. */
static void test_reader_preempts_writer(void)
{
  static AppLogTextRecord slots[4];
  AppLogServer server;
  memset(&server, 0, sizeof(server));
  AppLogRing_Init(&server.text, slots, sizeof(AppLogTextRecord), 4U);
  AppLogRing_Init(&server.telemetry, slots, sizeof(AppLogTextRecord), 4U);
  server.ready = 1U;
  for (uint32_t i = 0; i < 3U; ++i) AppLogText_Append(&server.text, i, "old", 3U);
  /* Writer is mid-append of index 3 (slot 3, never readable yet) and the
   * ring is full: the oldest readable index must not be its slot. */
  AppLogTextRecord *in_progress = (AppLogTextRecord *)AppLogRing_Begin(&server.text);
  in_progress->seq = 0xDEADU;  /* half written */
  size_t length = 0U;
  InfrabotLogReplyHeader h = ask(&server, INFRABOT_LOG_OP_TEXT, 0U, 0U, 200U, 0U, &length);
  assert(h.cursor == 0U && h.record_count == 3U && h.next_cursor == 3U);
  /* Commit, then the ring has wrapped once: index 0 is gone, 1..3 readable. */
  in_progress->seq = 3U; in_progress->length = 3U; memcpy(in_progress->text, "new", 3U);
  AppLogRing_Commit(&server.text);
  /* Writer starts index 4 in slot 0, which used to be index 0. */
  AppLogTextRecord *next = (AppLogTextRecord *)AppLogRing_Begin(&server.text);
  next->seq = 0xBEEFU;
  h = ask(&server, INFRABOT_LOG_OP_TEXT, 0U, 0U, 201U, 0U, &length);
  assert(h.cursor == 1U && h.record_count == 3U && h.next_cursor == 4U);
  uint32_t seq;
  memcpy(&seq, reply + sizeof(h), 4);
  assert(seq == 1U);
}

/* Which arm states may download: disabled, idle hold, and the stop/fault states. */
static void test_download_policy(void)
{
  for (uint32_t status = 0U; status < 64U; ++status)
    assert(AppLog_DownloadAllowed((uint8_t)status) ==
           (status == INFRABOT_STATUS_DISABLED || status == INFRABOT_STATUS_IDLE_HOLD ||
            status == INFRABOT_STATUS_STOPPING || status == INFRABOT_STATUS_FAULT ||
            status == INFRABOT_STATUS_ESTOP));
  assert(AppLog_DownloadAllowed(INFRABOT_STATUS_DISABLED));
  assert(AppLog_DownloadAllowed(INFRABOT_STATUS_IDLE_HOLD));
  assert(!AppLog_DownloadAllowed(INFRABOT_STATUS_ENABLING));
  assert(!AppLog_DownloadAllowed(INFRABOT_STATUS_RT_JOINT));
  assert(!AppLog_DownloadAllowed(INFRABOT_STATUS_NRT_CARTESIAN));
  assert(!AppLog_DownloadAllowed(INFRABOT_STATUS_JOINT_ADMITTANCE));
  assert(!AppLog_DownloadAllowed(INFRABOT_STATUS_RECOVERY));
  assert(AppLog_DownloadAllowed(INFRABOT_STATUS_STOPPING));
  assert(AppLog_DownloadAllowed(INFRABOT_STATUS_FAULT));
  assert(AppLog_DownloadAllowed(INFRABOT_STATUS_ESTOP));
}

/* A fault latches the telemetry ring once and keeps it until a served RESUME. */
static void test_fault_latch(void)
{
  static AppLogTextRecord text_slots[8];
  static InfrabotTelemetryV3 telemetry_slots[11];
  AppLogServer server;
  memset(&server, 0, sizeof(server));
  AppLogRing_Init(&server.text, text_slots, sizeof(AppLogTextRecord), 8U);
  AppLogRing_Init(&server.telemetry, telemetry_slots, sizeof(InfrabotTelemetryV3), 11U);
  server.ready = 1U;
  size_t length = 0U;

  assert(!AppLog_IsFault(INFRABOT_STATUS_DISABLED, INFRABOT_STOP_NONE));
  assert(!AppLog_IsFault(INFRABOT_STATUS_IDLE_HOLD, INFRABOT_STOP_NONE));
  assert(!AppLog_IsFault(INFRABOT_STATUS_RT_JOINT, INFRABOT_STOP_NONE));
  assert(!AppLog_IsFault(INFRABOT_STATUS_STOPPING, INFRABOT_STOP_REQUESTED));
  assert(AppLog_IsFault(INFRABOT_STATUS_FAULT, INFRABOT_STOP_NONE));
  assert(AppLog_IsFault(INFRABOT_STATUS_ESTOP, INFRABOT_STOP_NONE));
  assert(AppLog_IsFault(INFRABOT_STATUS_STOPPING, INFRABOT_STOP_STREAM_LOST));
  assert(AppLog_IsFault(INFRABOT_STATUS_IDLE_HOLD, INFRABOT_STOP_MOVE_LOST));
  assert(AppLog_IsFault(INFRABOT_STATUS_STOPPING, INFRABOT_STOP_DRIVE_FAULT));
  assert(AppLog_IsFault(INFRABOT_STATUS_STOPPING, INFRABOT_STOP_TORQUE_LIMIT));

  /* Healthy ticks and the operator's own STOP never latch. */
  AppLog_FaultTick(&server, INFRABOT_STATUS_IDLE_HOLD, INFRABOT_STOP_NONE);
  AppLog_FaultTick(&server, INFRABOT_STATUS_STOPPING, INFRABOT_STOP_REQUESTED);
  assert(server.latched == 0U);
  AppLog_FaultTick(&server, INFRABOT_STATUS_IDLE_HOLD, INFRABOT_STOP_NONE);

  /* The first fault tick latches.  It is not a download freeze and never expires. */
  AppLog_FaultTick(&server, INFRABOT_STATUS_STOPPING, INFRABOT_STOP_STREAM_LOST);
  assert(server.latched == 1U && server.frozen == 0U);
  InfrabotLogReplyHeader h = ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 10U, 0U, &length);
  assert(h.frozen == 2U);
  (void)ask(&server, INFRABOT_LOG_OP_FREEZE, 0U, 0U, 11U, 0U, &length);
  h = ask(&server, INFRABOT_LOG_OP_STATUS, 0U, 0U, 12U, 0U, &length);
  assert(h.frozen == 3U);
  AppLog_ExpireFreeze(&server, 12U + APP_LOG_FREEZE_TIMEOUT_MS + 1U);
  assert(server.frozen == 0U && server.latched == 1U);
  AppLog_ExpireFreeze(&server, 600000U);
  assert(server.latched == 1U);

  /* A refused request (arm not idle) cannot release the latch. */
  h = ask(&server, INFRABOT_LOG_OP_RESUME, 0U, 0U, 20U, 1U, &length);
  assert(h.result == INFRABOT_LOG_BUSY && server.latched == 1U);
  h = ask(&server, INFRABOT_LOG_OP_RESUME, 0U, 0U, 21U, 0U, &length);
  assert(h.result == INFRABOT_LOG_OK && h.frozen == 0U && server.latched == 0U && server.frozen == 0U);

  /* The stop reason is still held after the RESUME: no second latch. */
  AppLog_FaultTick(&server, INFRABOT_STATUS_IDLE_HOLD, INFRABOT_STOP_STREAM_LOST);
  AppLog_FaultTick(&server, INFRABOT_STATUS_IDLE_HOLD, INFRABOT_STOP_STREAM_LOST);
  assert(server.latched == 0U);
  /* It clears (mode switch, enable); a new fault then latches again. */
  AppLog_FaultTick(&server, INFRABOT_STATUS_RT_JOINT, INFRABOT_STOP_NONE);
  assert(server.latched == 0U);
  AppLog_FaultTick(&server, INFRABOT_STATUS_FAULT, INFRABOT_STOP_DRIVE_FAULT);
  assert(server.latched == 1U);
}

int main(void)
{
  test_lines();
  test_ring_and_prune();
  test_server();
  test_reader_preempts_writer();
  test_download_policy();
  test_fault_latch();
  puts("run log: line split, ring wrap/prune, DMLG paging/clamp/rate/busy/freeze, writer preemption, download whitelist, fault latch: OK");
  return 0;
}
