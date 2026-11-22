/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_PERSIST_H
#define APP_PERSIST_H

#include <stdint.h>

/* Load/save the MCU-side calibration marker and arm control profile in the
 * reserved STM32H743 sector.  The motor-side 0xFE zero is still stored by each
 * DM drive; this record keeps the MCU coordinate/status and 1 kHz gains. */
uint8_t AppPersist_Busy(void);
uint8_t AppPersist_LoadZero(void);
uint8_t AppPersist_SaveZero(void);

/* The control profile is stored in the same reserved sector as the zero
 * marker, so saving either record preserves the other one. */
uint8_t AppPersist_LoadControlProfile(void);
uint8_t AppPersist_SaveControlProfile(void);

/* Static IPv4 settings, octets in dotted order (192.168.0.50 = {192,168,0,50}). */
typedef struct
{
  uint8_t ip[4];
  uint8_t netmask[4];
  uint8_t gateway[4];
} AppNetConfig;

/* The network record is a separate 32-byte record in the same sector, after
 * the control record; every save rewrites both, so neither loses the other.
 * Load returns 1 and fills *config when a valid record is stored.  Save
 * refuses an invalid address (AppPersist_NetworkValid) and returns 0 if
 * another task is writing the sector at that moment; the sector erase takes
 * about a second, so call it from a low-priority task. */
uint8_t AppPersist_LoadNetwork(AppNetConfig *config);
uint8_t AppPersist_SaveNetwork(const AppNetConfig *config);

/* Basic sanity: unicast host address, contiguous non-empty netmask, host
 * part neither all-zero nor all-one, gateway 0.0.0.0 or inside the subnet. */
uint8_t AppPersist_NetworkValid(const AppNetConfig *config);

/* LCD language (4.3in UI), a 32-byte record after the network record; every
 * save rewrites all three records of the sector, so none loses another.
 * Load returns 1 and fills *language when a valid record is stored.  Save
 * returns 0 when the sector is busy or the write fails; like the network
 * save it erases the sector (about a second), so call it from a low-priority
 * task with every drive disabled. */
enum { APP_UI_LANGUAGE_EN, APP_UI_LANGUAGE_ZH, APP_UI_LANGUAGES };
uint8_t AppPersist_LoadLanguage(uint8_t *language);
uint8_t AppPersist_SaveLanguage(uint8_t language);

#endif /* APP_PERSIST_H */
