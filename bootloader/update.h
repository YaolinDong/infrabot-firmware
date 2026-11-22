/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef INFRABOT_NETBOOT_H
#define INFRABOT_NETBOOT_H
#include <stddef.h>
#include <stdint.h>
#define FW_META_ADDR  0x08020000U
#define FW_APP_ADDR   0x08020400U
#define FW_CFG_ADDR   0x081e0000U
#define FW_B_META 0x08100000U
#define FW_B_APP (FW_B_META+1024U)
#define FW_CAPACITY (0xe0000U-1024U)
#define FW_SLOT_A(meta) ((meta)==FW_META_ADDR)
#define FW_OTHER(meta) (FW_SLOT_A(meta)?FW_B_META:FW_META_ADDR)
#define FW_ATTEMPT 0x74726931U
#define FW_CONFIRM_REQ 0x636f6e31U
#define FW_REVERT_REQ 0x72657631U
#define FW_INSTALLED_REQ 0x69737632U
#define FW_FAULT_REQ 0x66617531U
#define FW_PORT 5002
#define FW_CHUNK 1024U
#ifndef FW_MAILBOX
#define FW_MAILBOX ((volatile uint32_t *)0x20000000U)
#endif
#define FW_REQUEST_MAGIC 0xcb74210eU
enum { FW_HELLO=1, FW_ENTER, FW_BEGIN, FW_DATA, FW_COMMIT, FW_CONFIRM, FW_REVERT };
enum { FW_OK, FW_BUSY, FW_INVALID, FW_SESSION, FW_OFFSET, FW_AUTH, FW_FLASH, FW_RNG, FW_PENDING, FW_DENIED, FW_EXPIRED };
enum { FW_APPLICATION=1, FW_LOADER=2, FW_TRIAL=3 };
typedef struct {
  char magic[8]; uint32_t base,length; uint8_t release_id[16],tag[32];
} FwManifest;
typedef struct {
  char magic[4]; uint8_t version,op,status,role; uint64_t session;
  uint32_t seq,offset;
} FwHeader;
_Static_assert(sizeof(FwManifest)==64 && offsetof(FwManifest,tag)==32,"IFW manifest layout");
_Static_assert(sizeof(FwHeader)==24 && offsetof(FwHeader,session)==8,"IFW packet layout");
static inline void fw_mail(uint32_t op,uint32_t base,uint32_t gen){
 FW_MAILBOX[0]=0;FW_MAILBOX[2]=base;FW_MAILBOX[3]=gen;
 FW_MAILBOX[1]=~op;FW_MAILBOX[0]=op;
}
/* Last four DTCM mailbox words are reserved for a physical SWD host.
 * The application publishes readiness; only the control board requests reset. */
#define FW_SWD_TRIAL_READY 0x31545753U
#define FW_SWD_NORMAL_READY 0x314e5753U
#define FW_SWD_ACCEPT      0x31435753U
void fw_swd_poll(void);
int fw_is_trial(void);
uint32_t fw_running_base(void);
/* Single approved transaction, carried across software reset in reserved DTCM.
 * DTCM is not initialized by the application startup. Loader clears on cold boot. */
typedef struct {uint64_t cookie;uint32_t ip,port;uint8_t tag[32];} FwTicket;
#define FW_TICKET ((volatile FwTicket *)(FW_MAILBOX+8))
/* Last 16 bytes of the 96-byte reserved mailbox. Diagnostics only. */
#define FW_BOOT_STATS (FW_MAILBOX+20)
#define FW_BOOT_STATS_MAGIC 0x53434e32U
_Static_assert(sizeof(FwTicket)<=(20U-8U)*sizeof(uint32_t),"ticket overlaps boot diagnostics");
int fw_manifest_ok(const FwManifest *m);
int fw_image_ok(const FwManifest *m,const uint8_t *image);
int fw_service_start(void);
#endif
