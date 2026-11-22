/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef FW_STAGED_SHA_H
#define FW_STAGED_SHA_H
#include <stdint.h>
#include <stddef.h>
int fw_staged_sha(const uint8_t *image,size_t length,uint8_t digest[32]);
#endif
