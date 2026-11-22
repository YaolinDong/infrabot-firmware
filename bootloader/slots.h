/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef INFRABOT_SLOTS_H
#define INFRABOT_SLOTS_H
#include "update.h"
#define FW_READY_MARK   0x59444152U
#define FW_TRIED_MARK   0x44454952U
#define FW_GOOD_MARK    0x444f4f47U
#define FW_REVOKE_MARK  0x4f564552U
typedef struct {uint32_t magic,generation,inverse,base;uint8_t tag[16];} FwMark;
_Static_assert(sizeof(FwMark)==32,"one H7 flashword");
typedef struct {uint32_t base,gen;int valid,good,tried,revoked;} FwSlot;
extern FwSlot fw_slots[2];
extern int fw_active_slot;
void fw_progress(void);
void fw_guard(unsigned slot);
int fw_select_start(uint32_t reason,uint32_t base,uint32_t gen,int recovery,int *trial);
int fw_boot_image_ok(const FwManifest *m,const uint8_t *image);
/* Return the number of full-image verification calls, not valid slots. */
unsigned fw_scan_slots(unsigned skipped,int ordinary_boot);
int fw_known(void);
int fw_candidate(int known);
uint32_t fw_destination(void);
int fw_mark(uint32_t base,unsigned offset,uint32_t type,uint32_t generation);
#endif
