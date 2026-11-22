/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "app_persist.h"

#include "app_config.h"
#include "app_state.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* STM32H743 2-MB parts have eight 128-KB sectors in bank 2.  The linker
 * script reserves the final sector so application code can never overlap it.
 * Zero calibration and the MCU-side control profile intentionally share one
 * record: erasing one must never silently erase the other. */
#ifndef DM_ZERO_FLASH_ADDRESS   /* host tests place the sector in RAM */
#define DM_ZERO_FLASH_ADDRESS  ((uint32_t)0x081E0000U)
#endif
#define DM_ZERO_FLASH_SECTOR   FLASH_SECTOR_7
#define DM_ZERO_FLASH_BANK     FLASH_BANK_2
#define DM_ZERO_FLASH_MAGIC    ((uint32_t)0x444D5A30U) /* "DMZ0" */
#define DM_ZERO_FLASH_VERSION  ((uint16_t)3U)
#define DM_ZERO_FLASH_OLD_VERSION ((uint16_t)2U)
#define DM_ZERO_OFFSET_LIMIT   (100.0f)
/* Network record: the flash word right after the 256-byte control record. */
#define DM_NET_FLASH_ADDRESS   (DM_ZERO_FLASH_ADDRESS + 256U)
#define DM_NET_FLASH_MAGIC     ((uint32_t)0x444D4E30U) /* "DMN0" */
#define DM_NET_FLASH_VERSION   ((uint16_t)1U)
/* UI settings record: the flash word after the network record. */
#define DM_UI_FLASH_ADDRESS    (DM_NET_FLASH_ADDRESS + 32U)
#define DM_UI_FLASH_MAGIC      ((uint32_t)0x444D5530U) /* "DMU0" */
#define DM_UI_FLASH_VERSION    ((uint16_t)1U)
/* Everything a save reads back and rewrites: control, network, UI. */
#define DM_PERSIST_SPAN        (256U + 32U + 32U)

/* Values consumed by the MCU's 1 kHz controller.  These are distinct from
 * the motor's own parameter Flash and are persisted by the new DMCP command. */
typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t motor_count;
  uint32_t sequence;
  float offsets[DM_MOTOR_COUNT];
  float kp[DM_MOTOR_COUNT];
  float kd[DM_MOTOR_COUNT];
  float velocity[DM_MOTOR_COUNT];
  float torque[DM_MOTOR_COUNT];
  float velocity_limit[DM_MOTOR_COUNT];
  float torque_ratio[DM_MOTOR_COUNT];
  uint8_t mode[DM_MOTOR_COUNT];
  uint8_t reserved0[4];
  uint32_t crc32;
  uint32_t reserved[1];
} DM_ControlFlashRecord_t;

/* Version 2 was the old zero-only record.  Keep it readable so upgrading the
 * firmware does not lose an already calibrated arm. */
typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t motor_count;
  uint32_t sequence;
  float offsets[DM_MOTOR_COUNT];
  uint32_t crc32;
  uint32_t reserved[4];
} DM_OldZeroFlashRecord_t;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t reserved0;
  uint8_t ip[4];
  uint8_t netmask[4];
  uint8_t gateway[4];
  uint32_t sequence;
  uint32_t reserved1;
  uint32_t crc32;
} DM_NetFlashRecord_t;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint8_t language;      /* APP_UI_LANGUAGE_* */
  uint8_t reserved0[17];
  uint32_t sequence;
  uint32_t crc32;
} DM_UiFlashRecord_t;

_Static_assert(sizeof(DM_NetFlashRecord_t) == 32U, "network record must be one flash word");
_Static_assert(sizeof(DM_UiFlashRecord_t) == 32U, "UI record must be one flash word");
_Static_assert(sizeof(DM_ControlFlashRecord_t) == 256U,
               "control record must be eight STM32H7 flash words");
_Static_assert(sizeof(DM_OldZeroFlashRecord_t) == 64U,
               "old zero record must be two STM32H7 flash words");

static uint32_t AppPersist_Crc32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
      crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
  }
  return ~crc;
}

static void AppPersist_ReadRaw(void *record, size_t length)
{
  /* The H7 D-cache can retain a line read before an erase/program operation. */
  SCB_InvalidateDCache_by_Addr((volatile void *)DM_ZERO_FLASH_ADDRESS, DM_PERSIST_SPAN);
  memcpy(record, (const void *)DM_ZERO_FLASH_ADDRESS, length);
}

static uint8_t AppPersist_OldRecordValid(const DM_OldZeroFlashRecord_t *record)
{
  if ((record == NULL) || (record->magic != DM_ZERO_FLASH_MAGIC) ||
      (record->version != DM_ZERO_FLASH_OLD_VERSION) ||
      (record->motor_count != DM_MOTOR_COUNT) || (record->sequence == 0U))
    return 0U;
  if (record->crc32 != AppPersist_Crc32((const uint8_t *)record,
                                        offsetof(DM_OldZeroFlashRecord_t, crc32)))
    return 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    if (!isfinite(record->offsets[i]) || record->offsets[i] < -DM_ZERO_OFFSET_LIMIT ||
        record->offsets[i] > DM_ZERO_OFFSET_LIMIT) return 0U;
  return 1U;
}

static uint8_t AppPersist_ModeValid(uint8_t index, uint8_t mode)
{
  if (index >= DM_MOTOR_COUNT) return 0U;
  return (index == DM_GRIPPER_INDEX) ?
      (mode == DM_ETH_MODE_FORCE_POS) :
      (mode == DM_ETH_MODE_MIT || mode == DM_ETH_MODE_POS_VEL ||
       mode == DM_ETH_MODE_VEL);
}

static uint8_t AppPersist_RecordValid(const DM_ControlFlashRecord_t *record)
{
  if ((record == NULL) || (record->magic != DM_ZERO_FLASH_MAGIC) ||
      (record->version != DM_ZERO_FLASH_VERSION) ||
      (record->motor_count != DM_MOTOR_COUNT) || (record->sequence == 0U))
    return 0U;
  if (record->crc32 != AppPersist_Crc32((const uint8_t *)record,
                                        offsetof(DM_ControlFlashRecord_t, crc32)))
    return 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    if (!isfinite(record->offsets[i]) || record->offsets[i] < -DM_ZERO_OFFSET_LIMIT ||
        record->offsets[i] > DM_ZERO_OFFSET_LIMIT ||
        !isfinite(record->kp[i]) || record->kp[i] < 0.0f || record->kp[i] > 500.0f ||
        !isfinite(record->kd[i]) || record->kd[i] < 0.0f || record->kd[i] > 5.0f ||
        !isfinite(record->velocity[i]) || fabsf(record->velocity[i]) > dm_motor_velocity_max[i] ||
        !isfinite(record->torque[i]) || fabsf(record->torque[i]) > dm_motor_torque_max[i] ||
        !isfinite(record->velocity_limit[i]) || record->velocity_limit[i] < 0.0f ||
        record->velocity_limit[i] > dm_motor_velocity_max[i] ||
        !isfinite(record->torque_ratio[i]) || record->torque_ratio[i] < 0.0f ||
        record->torque_ratio[i] > 1.0f || !AppPersist_ModeValid(i, record->mode[i]))
      return 0U;
  }
  return 1U;
}

static void AppPersist_BuildCurrent(DM_ControlFlashRecord_t *record)
{
  memset(record, 0, sizeof(*record));
  record->magic = DM_ZERO_FLASH_MAGIC;
  record->version = DM_ZERO_FLASH_VERSION;
  record->motor_count = DM_MOTOR_COUNT;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    record->offsets[i] = dm_motor_runtime.zero_offset[i];
    record->kp[i] = eth_udp_command.target_kp[i];
    record->kd[i] = eth_udp_command.target_kd[i];
    record->velocity[i] = eth_udp_command.target_velocity[i];
    record->torque[i] = eth_udp_command.target_torque[i];
    record->velocity_limit[i] = eth_udp_command.velocity_limit[i];
    record->torque_ratio[i] = eth_udp_command.torque_limit_ratio[i];
    record->mode[i] = eth_udp_command.motor_mode[i];
  }
}

static uint8_t AppPersist_ReadCurrent(DM_ControlFlashRecord_t *record,
                                      DM_OldZeroFlashRecord_t *old_record)
{
  uint8_t raw[256];
  AppPersist_ReadRaw(raw, sizeof(raw));
  memcpy(record, raw, sizeof(*record));
  if (AppPersist_RecordValid(record) != 0U) return 1U;
  memcpy(old_record, raw, sizeof(*old_record));
  return 0U;
}

static uint32_t AppPersist_PreviousSequence(const DM_ControlFlashRecord_t *record,
                                            const DM_OldZeroFlashRecord_t *old_record,
                                            uint8_t record_valid,
                                            uint8_t old_valid)
{
  if (record_valid != 0U) return record->sequence;
  if (old_valid != 0U) return old_record->sequence;
  return 0U;
}

/* Only one task may erase/program the sector at a time.  A second writer
 * fails at once instead of waiting through a ~1 s erase (the control task
 * must never block on the LCD task, or the other way round). */
static volatile uint8_t app_persist_busy;
uint8_t AppPersist_Busy(void){return app_persist_busy;}

static uint8_t AppPersist_TryLock(void)
{
  uint8_t taken = 0U;
  taskENTER_CRITICAL();
  if (app_persist_busy == 0U)
  {
    app_persist_busy = 1U;
    taken = 1U;
  }
  taskEXIT_CRITICAL();
  if (taken == 0U) printf("PERSIST: flash busy (another save in progress)\r\n");
  return taken;
}

static void AppPersist_Unlock(void)
{
  app_persist_busy = 0U;
}

static uint8_t AppPersist_NetRecordValid(const DM_NetFlashRecord_t *record)
{
  if ((record->magic != DM_NET_FLASH_MAGIC) || (record->version != DM_NET_FLASH_VERSION) ||
      (record->crc32 != AppPersist_Crc32((const uint8_t *)record,
                                         offsetof(DM_NetFlashRecord_t, crc32))))
    return 0U;
  AppNetConfig config;
  memcpy(config.ip, record->ip, 4U);
  memcpy(config.netmask, record->netmask, 4U);
  memcpy(config.gateway, record->gateway, 4U);
  return AppPersist_NetworkValid(&config);
}

static void AppPersist_ReadNet(DM_NetFlashRecord_t *record)
{
  uint8_t raw[256 + sizeof(DM_NetFlashRecord_t)];
  AppPersist_ReadRaw(raw, sizeof(raw));
  memcpy(record, raw + 256, sizeof(*record));
}

static uint8_t AppPersist_UiRecordValid(const DM_UiFlashRecord_t *record)
{
  return (record->magic == DM_UI_FLASH_MAGIC) && (record->version == DM_UI_FLASH_VERSION) &&
         (record->language < APP_UI_LANGUAGES) &&
         (record->crc32 == AppPersist_Crc32((const uint8_t *)record, offsetof(DM_UiFlashRecord_t, crc32)));
}

static void AppPersist_ReadUi(DM_UiFlashRecord_t *record)
{
  uint8_t raw[DM_PERSIST_SPAN];
  AppPersist_ReadRaw(raw, sizeof(raw));
  memcpy(record, raw + 256U + 32U, sizeof(*record));
}

/* Erases the sector and programs the 256-byte control area (`control`, raw
 * bytes as read or built; NULL leaves it erased), the network record (`net`)
 * and the UI record (`ui`; NULL leaves each erased), then reads them back. */
static uint8_t AppPersist_WriteSector(const void *control, const DM_NetFlashRecord_t *net,
                                      const DM_UiFlashRecord_t *ui)
{
  HAL_StatusTypeDef status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    printf("PERSIST: flash unlock failed hal=0x%08lX\r\n",
           (unsigned long)HAL_FLASH_GetError());
    return 0U;
  }
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t erase_error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = DM_ZERO_FLASH_BANK;
  erase.Sector = DM_ZERO_FLASH_SECTOR;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  status = HAL_FLASHEx_Erase(&erase, &erase_error);
  uint8_t flash_word[32] __attribute__((aligned(32)));
  for (uint32_t offset = 0U; (status == HAL_OK) && (control != NULL) && (offset < 256U);
       offset += sizeof(flash_word))
  {
    memcpy(flash_word, ((const uint8_t *)control) + offset, sizeof(flash_word));
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DM_ZERO_FLASH_ADDRESS + offset,
                               (uint32_t)flash_word);
  }
  if ((status == HAL_OK) && (net != NULL))
  {
    memcpy(flash_word, net, sizeof(flash_word));
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DM_NET_FLASH_ADDRESS,
                               (uint32_t)flash_word);
  }
  if ((status == HAL_OK) && (ui != NULL))
  {
    memcpy(flash_word, ui, sizeof(flash_word));
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DM_UI_FLASH_ADDRESS,
                               (uint32_t)flash_word);
  }
  (void)HAL_FLASH_Lock();
  if (status == HAL_OK)
  {
    uint8_t verify[DM_PERSIST_SPAN];
    AppPersist_ReadRaw(verify, sizeof(verify));
    if (((control != NULL) && (memcmp(verify, control, 256U) != 0)) ||
        ((net != NULL) && (memcmp(verify + 256, net, sizeof(*net)) != 0)) ||
        ((ui != NULL) && (memcmp(verify + 256 + 32, ui, sizeof(*ui)) != 0)))
      status = HAL_ERROR;
  }
  if (status != HAL_OK)
  {
    printf("PERSIST: flash save failed hal=0x%08lX erase=0x%08lX\r\n",
           (unsigned long)HAL_FLASH_GetError(), (unsigned long)erase_error);
    return 0U;
  }
  return 1U;
}

uint8_t AppPersist_LoadZero(void)
{
  DM_ControlFlashRecord_t record;
  DM_OldZeroFlashRecord_t old_record;
  const uint8_t current_valid = AppPersist_ReadCurrent(&record, &old_record);
  const uint8_t old_valid = (current_valid == 0U) ?
      AppPersist_OldRecordValid(&old_record) : 0U;
  if (current_valid == 0U && old_valid == 0U)
  {
    dm_motor_runtime.zero_valid = 0U;
    printf("ZERO: no valid flash record at 0x%08lX; waiting for PC SET_ZERO\r\n",
           (unsigned long)DM_ZERO_FLASH_ADDRESS);
    return 0U;
  }
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    dm_motor_runtime.zero_offset[i] = (current_valid != 0U) ? record.offsets[i] : old_record.offsets[i];
    eth_udp_command.target_position[i] = 0.0f;
  }
  dm_motor_runtime.zero_valid = 1U;
  printf("ZERO: restored flash record seq=%lu offsets_mrad=",
         (unsigned long)((current_valid != 0U) ? record.sequence : old_record.sequence));
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    printf("%ld%s", (long)(dm_motor_runtime.zero_offset[i] * 1000.0f),
           (i + 1U == DM_MOTOR_COUNT) ? "" : ",");
  printf("\r\n");
  return 1U;
}

uint8_t AppPersist_LoadControlProfile(void)
{
  DM_ControlFlashRecord_t record;
  DM_OldZeroFlashRecord_t old_record;
  if (AppPersist_ReadCurrent(&record, &old_record) == 0U ||
      AppPersist_RecordValid(&record) == 0U)
  {
    printf("CTRL_PROFILE: no valid MCU Flash profile; using compiled defaults\r\n");
    return 0U;
  }
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    eth_udp_command.motor_mode[i] = record.mode[i];
    eth_udp_command.target_kp[i] = record.kp[i];
    eth_udp_command.target_kd[i] = record.kd[i];
    eth_udp_command.target_velocity[i] = record.velocity[i];
    eth_udp_command.target_torque[i] = record.torque[i];
    eth_udp_command.velocity_limit[i] = record.velocity_limit[i];
    eth_udp_command.torque_limit_ratio[i] = record.torque_ratio[i];
  }
  printf("CTRL_PROFILE: restored MCU Flash seq=%lu Kp/Kd=",
         (unsigned long)record.sequence);
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    printf("%.3f/%.3f%s", record.kp[i], record.kd[i],
           (i + 1U == DM_MOTOR_COUNT) ? "" : ",");
  printf("\r\n");
  return 1U;
}

static uint8_t AppPersist_SaveWithCurrentProfile(void)
{
  DM_ControlFlashRecord_t record;
  DM_ControlFlashRecord_t previous;
  DM_OldZeroFlashRecord_t old_record;
  const uint8_t previous_valid = AppPersist_ReadCurrent(&previous, &old_record);
  const uint8_t old_valid = (previous_valid == 0U) ?
      AppPersist_OldRecordValid(&old_record) : 0U;
  AppPersist_BuildCurrent(&record);
  if ((previous_valid == 0U) && (old_valid != 0U))
    memcpy(record.offsets, old_record.offsets, sizeof(record.offsets));
  const uint32_t previous_sequence = AppPersist_PreviousSequence(
      &previous, &old_record, previous_valid, old_valid);
  record.sequence = previous_sequence + 1U;
  if (record.sequence == 0U) record.sequence = 1U;
  record.crc32 = AppPersist_Crc32((const uint8_t *)&record,
                                  offsetof(DM_ControlFlashRecord_t, crc32));
  if (AppPersist_RecordValid(&record) == 0U)
  {
    printf("CTRL_PROFILE: refusing invalid values\r\n");
    return 0U;
  }
  DM_NetFlashRecord_t net;
  DM_UiFlashRecord_t ui;
  AppPersist_ReadNet(&net);
  AppPersist_ReadUi(&ui);
  if (AppPersist_TryLock() == 0U) return 0U;
  const uint8_t written = AppPersist_WriteSector(&record,
                                                 AppPersist_NetRecordValid(&net) ? &net : NULL,
                                                 AppPersist_UiRecordValid(&ui) ? &ui : NULL);
  AppPersist_Unlock();
  if (written == 0U) return 0U;
  printf("CTRL_PROFILE: saved to MCU Flash seq=%lu address=0x%08lX\r\n",
         (unsigned long)record.sequence, (unsigned long)DM_ZERO_FLASH_ADDRESS);
  return 1U;
}

uint8_t AppPersist_SaveZero(void)
{
  /* Save the combined v3 record so calibration preserves the control profile. */
  return AppPersist_SaveWithCurrentProfile();
}

uint8_t AppPersist_SaveControlProfile(void)
{
  return AppPersist_SaveWithCurrentProfile();
}

uint8_t AppPersist_NetworkValid(const AppNetConfig *config)
{
  uint32_t ip = 0U, mask = 0U, gw = 0U;
  for (uint8_t i = 0U; i < 4U; i++)
  {
    ip = (ip << 8) | config->ip[i];
    mask = (mask << 8) | config->netmask[i];
    gw = (gw << 8) | config->gateway[i];
  }
  /* Contiguous mask: the inverted mask plus one is a power of two. */
  const uint32_t host = ~mask;
  if ((mask == 0U) || (host == 0U) || (((host + 1U) & host) != 0U)) return 0U;
  /* Unicast host: not 0.x, 127.x or class D/E. */
  if ((config->ip[0] == 0U) || (config->ip[0] == 127U) || (config->ip[0] >= 224U)) return 0U;
  if (((ip & host) == 0U) || ((ip & host) == host)) return 0U;
  if ((gw != 0U) && (((gw & mask) != (ip & mask)) || (gw == ip) ||
                     ((gw & host) == 0U) || ((gw & host) == host)))
    return 0U;
  return 1U;
}

uint8_t AppPersist_LoadNetwork(AppNetConfig *config)
{
  DM_NetFlashRecord_t record;
  AppPersist_ReadNet(&record);
  if (AppPersist_NetRecordValid(&record) == 0U) return 0U;
  memcpy(config->ip, record.ip, 4U);
  memcpy(config->netmask, record.netmask, 4U);
  memcpy(config->gateway, record.gateway, 4U);
  return 1U;
}

uint8_t AppPersist_SaveNetwork(const AppNetConfig *config)
{
  uint8_t control[256];
  DM_ControlFlashRecord_t current;
  DM_OldZeroFlashRecord_t old_record;
  DM_NetFlashRecord_t previous, record;

  if (AppPersist_NetworkValid(config) == 0U) return 0U;
  /* Keep whatever calibration/profile record is there (current or the old
   * zero-only layout) byte for byte. */
  AppPersist_ReadRaw(control, sizeof(control));
  const uint8_t control_valid = AppPersist_ReadCurrent(&current, &old_record) ||
                                AppPersist_OldRecordValid(&old_record);
  AppPersist_ReadNet(&previous);

  memset(&record, 0, sizeof(record));
  record.magic = DM_NET_FLASH_MAGIC;
  record.version = DM_NET_FLASH_VERSION;
  memcpy(record.ip, config->ip, 4U);
  memcpy(record.netmask, config->netmask, 4U);
  memcpy(record.gateway, config->gateway, 4U);
  record.sequence = AppPersist_NetRecordValid(&previous) ? previous.sequence + 1U : 1U;
  record.crc32 = AppPersist_Crc32((const uint8_t *)&record, offsetof(DM_NetFlashRecord_t, crc32));

  DM_UiFlashRecord_t ui;
  AppPersist_ReadUi(&ui);
  if (AppPersist_TryLock() == 0U) return 0U;
  const uint8_t written = AppPersist_WriteSector(control_valid ? control : NULL, &record,
                                                 AppPersist_UiRecordValid(&ui) ? &ui : NULL);
  AppPersist_Unlock();
  if (written != 0U)
    printf("NET: saved to MCU Flash seq=%lu IP=%u.%u.%u.%u\r\n", (unsigned long)record.sequence,
           config->ip[0], config->ip[1], config->ip[2], config->ip[3]);
  return written;
}

uint8_t AppPersist_LoadLanguage(uint8_t *language)
{
  DM_UiFlashRecord_t record;
  AppPersist_ReadUi(&record);
  if (AppPersist_UiRecordValid(&record) == 0U) return 0U;
  *language = record.language;
  return 1U;
}

uint8_t AppPersist_SaveLanguage(uint8_t language)
{
  if (language >= APP_UI_LANGUAGES) return 0U;
  /* Keep the control/zero area and the network record byte for byte. */
  uint8_t control[256];
  DM_ControlFlashRecord_t current;
  DM_OldZeroFlashRecord_t old_record;
  DM_NetFlashRecord_t net;
  DM_UiFlashRecord_t previous, record;
  AppPersist_ReadRaw(control, sizeof(control));
  const uint8_t control_valid = AppPersist_ReadCurrent(&current, &old_record) ||
                                AppPersist_OldRecordValid(&old_record);
  AppPersist_ReadNet(&net);
  AppPersist_ReadUi(&previous);
  if (AppPersist_UiRecordValid(&previous) && previous.language == language) return 1U;

  memset(&record, 0, sizeof(record));
  record.magic = DM_UI_FLASH_MAGIC;
  record.version = DM_UI_FLASH_VERSION;
  record.language = language;
  record.sequence = AppPersist_UiRecordValid(&previous) ? previous.sequence + 1U : 1U;
  record.crc32 = AppPersist_Crc32((const uint8_t *)&record, offsetof(DM_UiFlashRecord_t, crc32));

  if (AppPersist_TryLock() == 0U) return 0U;
  const uint8_t written = AppPersist_WriteSector(control_valid ? control : NULL,
                                                 AppPersist_NetRecordValid(&net) ? &net : NULL, &record);
  AppPersist_Unlock();
  if (written != 0U)
    printf("UI: language %u saved to MCU Flash seq=%lu\r\n", (unsigned)language, (unsigned long)record.sequence);
  return written;
}
