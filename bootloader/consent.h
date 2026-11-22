/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef FW_CONSENT_H
#define FW_CONSENT_H
#include <stdint.h>
typedef struct {uint64_t id; uint32_t ip; uint8_t op,tag[32];} FwPrompt;
int fw_consent_poll(uint64_t id,uint32_t ip,unsigned op,const uint8_t tag[32]);
int fw_consent_reserved(void);
int fw_consent_view(FwPrompt *out);
void fw_consent_decide(uint64_t id,int allow);
#endif
