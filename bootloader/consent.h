/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef FW_CONSENT_H
#define FW_CONSENT_H
#include <stdint.h>
/* FW_VERSION_LEN: requester-reported version of the new image, NUL-terminated.
 * It is not signed; the image tag remains the identity to check. */
#define FW_VERSION_LEN 24U
typedef struct {uint64_t id; uint32_t ip; uint8_t op,tag[32]; char version[FW_VERSION_LEN];} FwPrompt;
int fw_consent_poll(uint64_t id,uint32_t ip,unsigned op,const uint8_t tag[32],const char *version);
int fw_consent_reserved(void);
int fw_consent_view(FwPrompt *out);
void fw_consent_decide(uint64_t id,int allow);
#endif
