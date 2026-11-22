/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Read aligned image words once from uncached Flash into a 512-byte DTCM
 * stack buffer. mbedTLS does the complete SHA-256 (including any tail).
 * Do not enable D-cache for image reads or cross the signed byte range. */
#include "mbedtls/sha256.h"
#include <stdint.h>
#include "staged_sha.h"

int fw_staged_sha(const uint8_t *image, size_t length, uint8_t digest[32])
{
    mbedtls_sha256_context ctx;
    uint32_t words[128];
    typedef uint32_t alias_word __attribute__((may_alias));
    mbedtls_sha256_init(&ctx);
    int err=mbedtls_sha256_starts_ret(&ctx,0);
    while (!err && length>=sizeof words && !((uintptr_t)image&3U)) {
        const volatile alias_word *src=(const volatile alias_word *)(const void *)image;
        for (unsigned j=0;j<128;j++) words[j]=src[j];
        err=mbedtls_sha256_update_ret(&ctx,(const uint8_t *)words,sizeof words);
        image+=sizeof words; length-=sizeof words;
    }
    if(!err)err=mbedtls_sha256_update_ret(&ctx,image,length);
    if(!err)err=mbedtls_sha256_finish_ret(&ctx,digest);
    mbedtls_sha256_free(&ctx);
    return err;
}
