/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_log.h"

#include <string.h>

/* ---- Record ring -------------------------------------------------------------- */
void AppLogRing_Init(AppLogRing *ring, void *base, uint32_t record_size, uint32_t count)
{
  ring->base = (uint8_t *)base;
  ring->record_size = record_size;
  ring->count = count;
  ring->next = 0U;
  ring->kept = 0U;
}

void *AppLogRing_Slot(const AppLogRing *ring, uint32_t index)
{
  return ring->base + (size_t)(index % ring->count) * ring->record_size;
}

void *AppLogRing_Begin(AppLogRing *ring) { return AppLogRing_Slot(ring, ring->next); }

void AppLogRing_Commit(AppLogRing *ring)
{
  /* next first: a reader that runs between the two stores sees one record
   * fewer, never an unwritten one. */
  ring->next = ring->next + 1U;
  if (ring->kept < ring->count - 1U) ring->kept = ring->kept + 1U;
}

uint32_t AppLogRing_First(const AppLogRing *ring) { return ring->next - ring->kept; }

void AppLogRing_DropOldest(AppLogRing *ring)
{
  if (ring->kept != 0U) ring->kept = ring->kept - 1U;
}

/* ---- Text ---------------------------------------------------------------------- */
void AppLogText_Append(AppLogRing *ring, uint32_t time_ms, const char *text, uint32_t length)
{
  if (length > APP_LOG_TEXT_LINE_MAX) length = APP_LOG_TEXT_LINE_MAX;
  AppLogTextRecord *record = (AppLogTextRecord *)AppLogRing_Begin(ring);
  record->seq = ring->next;
  record->time_ms = time_ms;
  record->length = (uint16_t)length;
  memcpy(record->text, text, length);
  AppLogRing_Commit(ring);
}

void AppLogText_Prune(AppLogRing *ring, uint32_t now_ms, uint32_t retain_ms)
{
  while (ring->kept != 0U)
  {
    const AppLogTextRecord *oldest =
      (const AppLogTextRecord *)AppLogRing_Slot(ring, AppLogRing_First(ring));
    if ((uint32_t)(now_ms - oldest->time_ms) <= retain_ms) break;
    AppLogRing_DropOldest(ring);
  }
}

/* ---- Lines --------------------------------------------------------------------- */
int AppLogLine_Feed(AppLogLine *line, uint8_t byte)
{
  if (byte == '\r') return 0;
  if (byte == '\n') return 1;
  line->text[line->length++] = (char)byte;
  return line->length >= APP_LOG_TEXT_LINE_MAX;
}

/* ---- Requests ------------------------------------------------------------------ */
static uint32_t clamp_cursor(const AppLogRing *ring, uint32_t cursor)
{
  const uint32_t first = AppLogRing_First(ring);
  /* Wrapping distance from first: inside [first, next] when <= kept. */
  if ((uint32_t)(cursor - first) > ring->kept) return first;
  return cursor;
}

static size_t read_text(const AppLogRing *ring, uint32_t cursor, uint8_t *out, size_t capacity,
                        uint32_t *next_cursor, uint8_t *count)
{
  size_t used = 0U;
  uint32_t index = cursor;
  *count = 0U;
  while (index != ring->next && *count < 255U)
  {
    const AppLogTextRecord *record = (const AppLogTextRecord *)AppLogRing_Slot(ring, index);
    const uint32_t length = record->length <= APP_LOG_TEXT_LINE_MAX ? record->length : 0U;
    const size_t size = (INFRABOT_LOG_TEXT_HEADER + length + 3U) & ~(size_t)3U;
    if (record->seq != index || used + size > capacity) break;
    memcpy(out + used, record, INFRABOT_LOG_TEXT_HEADER + length);
    memset(out + used + INFRABOT_LOG_TEXT_HEADER + length, 0,
           size - INFRABOT_LOG_TEXT_HEADER - length);
    used += size;
    ++index;
    ++*count;
  }
  *next_cursor = index;
  return used;
}

static size_t read_records(const AppLogRing *ring, uint32_t cursor, uint8_t *out, size_t capacity,
                           uint32_t *next_cursor, uint8_t *count)
{
  size_t used = 0U;
  uint32_t index = cursor;
  *count = 0U;
  while (index != ring->next && used + ring->record_size <= capacity)
  {
    memcpy(out + used, AppLogRing_Slot(ring, index), ring->record_size);
    used += ring->record_size;
    ++index;
    ++*count;
  }
  *next_cursor = index;
  return used;
}

size_t AppLog_Serve(AppLogServer *server, const void *request_bytes, size_t request_size,
                    const AppLogContext *context, uint8_t *reply_bytes)
{
  InfrabotLogRequest request;
  if (request_size != sizeof(request)) return 0U;
  memcpy(&request, request_bytes, sizeof(request));
  if (request.magic != INFRABOT_MAGIC_LOG_REQUEST) return 0U;

  /* A flood gets no reply; the PC retries after its timeout. */
  if (server->serve_ms == context->now_ms)
  {
    if (server->serve_count >= APP_LOG_REPLIES_PER_MS) return 0U;
    server->serve_count++;
  }
  else
  {
    server->serve_ms = context->now_ms;
    server->serve_count = 1U;
  }

  InfrabotLogReplyHeader header;
  memset(&header, 0, sizeof(header));
  header.magic = INFRABOT_MAGIC_LOG_REPLY;
  header.version = INFRABOT_LOG_V1;
  header.op = request.op;
  header.request_id = request.request_id;
  header.now_ms = context->now_ms;
  header.now_us = context->now_us;
  uint8_t *data = reply_bytes + sizeof(header);

  if (request.version != INFRABOT_LOG_V1 || request.op > INFRABOT_LOG_OP_RESUME)
    header.result = INFRABOT_LOG_INVALID;
  else if (!server->ready)
    header.result = INFRABOT_LOG_UNAVAILABLE;
  else if (context->busy)
    header.result = INFRABOT_LOG_BUSY;
  else
  {
    header.result = INFRABOT_LOG_OK;
    server->last_request_ms = context->now_ms;
    const size_t capacity = (request.max_bytes == 0U || request.max_bytes > INFRABOT_LOG_DATA_MAX) ?
                            INFRABOT_LOG_DATA_MAX : request.max_bytes;
    uint32_t next_cursor = 0U;
    uint8_t count = 0U;
    size_t used = 0U;
    switch (request.op)
    {
      case INFRABOT_LOG_OP_TEXT:
        header.cursor = clamp_cursor(&server->text, request.cursor);
        used = read_text(&server->text, header.cursor, data, capacity, &next_cursor, &count);
        break;
      case INFRABOT_LOG_OP_TELEMETRY:
        header.cursor = clamp_cursor(&server->telemetry, request.cursor);
        used = read_records(&server->telemetry, header.cursor, data, capacity, &next_cursor, &count);
        break;
      case INFRABOT_LOG_OP_FREEZE: server->frozen = 1U; break;
      case INFRABOT_LOG_OP_RESUME: server->frozen = 0U; server->latched = 0U; break;
      default: /* STATUS */
        header.writer_max_us = server->writer_max_us;
        server->writer_max_us = 0U;
        break;
    }
    header.next_cursor = next_cursor;
    header.record_count = count;
    header.data_bytes = (uint16_t)used;
    header.boot_id = server->boot_id;
    header.text_first = AppLogRing_First(&server->text);
    header.text_next = server->text.next;
    header.telemetry_first = AppLogRing_First(&server->telemetry);
    header.telemetry_next = server->telemetry.next;
    header.frozen = (uint8_t)((server->frozen != 0U ? 1U : 0U) | (server->latched != 0U ? 2U : 0U));
    header.lost_bytes = server->lost_bytes;
  }
  memcpy(reply_bytes, &header, sizeof(header));
  return sizeof(header) + header.data_bytes;
}

void AppLog_ExpireFreeze(AppLogServer *server, uint32_t now_ms)
{
  if (server->frozen && (uint32_t)(now_ms - server->last_request_ms) > APP_LOG_FREEZE_TIMEOUT_MS)
    server->frozen = 0U;
}

uint8_t AppLog_IsFault(uint8_t status, uint8_t stop_reason)
{
  /* The operator's own STOP is not a fault; a lost stream or move keepalive is,
   * because its lead-up is what one wants to look at. */
  return (uint8_t)(status == INFRABOT_STATUS_FAULT || status == INFRABOT_STATUS_ESTOP ||
                   (stop_reason != INFRABOT_STOP_NONE && stop_reason != INFRABOT_STOP_REQUESTED));
}

void AppLog_FaultTick(AppLogServer *server, uint8_t status, uint8_t stop_reason)
{
  const uint8_t fault = AppLog_IsFault(status, stop_reason);
  /* Edge only: after a RESUME the arm may still hold the stop reason (it stays
   * until a mode switch or an enable), which must not latch the ring again. */
  if (fault != 0U && server->fault_prev == 0U) server->latched = 1U;
  server->fault_prev = fault;
}
