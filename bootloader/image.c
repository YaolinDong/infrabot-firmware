/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "update.h"
#include "fw_public.h"
#include "staged_sha.h"
#include "monocypher-ed25519.h"
#include "mbedtls/sha256.h"
#include <string.h>

/* Public trust anchor, also checked by factory/SWD tooling. No signing secret in flash. */
static const struct {uint32_t abi;uint8_t key[32];} trust
 __attribute__((used,section(".fw_api")))={0x32474146U,FW_PUBLIC_BYTES};

void mbedtls_platform_zeroize(void *ptr,size_t n){volatile uint8_t *p=ptr;while(n--)*p++=0;}
int fw_manifest_ok(const FwManifest *m){
 return !memcmp(m->magic,"IFH743v2",8) && (m->base==FW_APP_ADDR || m->base==FW_B_APP) &&
        m->length>=8+64 && m->length<=FW_CAPACITY;
}
static int verify_image(const FwManifest *m,const uint8_t *image,int cold_scan){
 uint32_t sp,pc;uint8_t digest[32];
 if(!fw_manifest_ok(m))return 0;
 uint32_t length=m->length-64;
 memcpy(&sp,image,4);memcpy(&pc,image+4,4);
 return sp==0x20020000U && (pc&1U) && (pc&~1U)>=m->base && (pc&~1U)-m->base<length &&
   !(cold_scan ? fw_staged_sha(image,length,digest) : mbedtls_sha256_ret(image,length,digest,0)) && !memcmp(digest,m->tag,32) &&
   crypto_ed25519_check(image+length,trust.key,(const uint8_t*)m,sizeof(*m))==0;
}

int fw_image_ok(const FwManifest *m,const uint8_t *image){return verify_image(m,image,0);}
int fw_boot_image_ok(const FwManifest *m,const uint8_t *image){return verify_image(m,image,1);}
