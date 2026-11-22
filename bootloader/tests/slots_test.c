/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"
#include "slots.h"
#include "staged_sha.h"
#include "mbedtls/sha256.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
extern int cut_at,operations;extern jmp_buf cut_env;
void map_flash(void);void load_factory(const char *);
int simulate_reset(uint32_t,uint32_t,uint32_t,int*);
int fw_install(const FwManifest*,const uint8_t*);
static uint8_t data[512],original[0xe0000],old_b[0xe0000];
static FwManifest manifest(const char *file){
 FwManifest m; FILE *f=fopen(file,"rb");assert(f);
 assert(fread(&m,1,64,f)==64 && m.length==sizeof(data) && fread(data,1,sizeof(data),f)==sizeof(data));
 fclose(f);return m;
}
void fw_progress(void){}
int main(int argc,char**argv){
 uint8_t storage[2112] __attribute__((aligned(4))),ref[32],digest[32];
 for(unsigned i=0;i<sizeof storage;i++)storage[i]=(uint8_t)(33U*i+i/13U);
 const unsigned lengths[]={0,1,55,56,63,64,127,508,509,511,512,513,1024,2049};
 for(unsigned offset=0;offset<4;offset++)for(unsigned j=0;j<sizeof lengths/sizeof *lengths;j++){
   assert(!fw_staged_sha(storage+offset,lengths[j],digest));
   assert(!mbedtls_sha256_ret(storage+offset,lengths[j],ref,0) && !memcmp(ref,digest,32));
 }
 assert(argc==4);map_flash();load_factory(argv[1]);
 memcpy(original,(void*)FW_META_ADDR,sizeof(original));
 int trial=0;FwManifest b=manifest(argv[3]);
 data[64]^=1;assert(!fw_install(&b,data)&&operations==0);data[64]^=1;
 data[sizeof(data)-1]^=1;assert(!fw_install(&b,data)&&operations==0);data[sizeof(data)-1]^=1;
 assert(fw_install(&b,data));int count=operations;
 assert(simulate_reset(0,0,0,&trial)==1 && trial);
 assert(simulate_reset(0,0,0,&trial)==0 && !trial); /* watchdog/power loss */
 for(int cut=1;cut<=count+2;cut++){
   load_factory(argv[1]);cut_at=cut;
   if(!setjmp(cut_env)){
      assert(fw_install(&b,data));assert(simulate_reset(0,0,0,&trial)==1&&trial);
      simulate_reset(FW_CONFIRM_REQ,FW_B_META,2,&trial);
   }
   cut_at=0;
   assert(!memcmp(original,(void*)FW_META_ADDR,sizeof(original)));
   assert(simulate_reset(0,0,0,&trial)==0&&!trial);
 }
 load_factory(argv[1]);assert(fw_install(&b,data));
 assert(simulate_reset(0,0,0,&trial)==1 && trial);
 assert(simulate_reset(FW_CONFIRM_REQ,FW_B_META,2,&trial)==1&&!trial);
 assert(simulate_reset(0,0,0,&trial)==1&&!trial);
 /* Both slots are GOOD. Account for calls to the real authenticator and
  * verify that changing either payload (GOOD marker still intact) matters. */
 assert(fw_scan_slots(0,1)==1 && !fw_slots[0].valid && fw_slots[1].valid && fw_known()==1);
 assert(fw_scan_slots(0,0)==2 && fw_slots[0].valid && fw_slots[1].valid); /* recovery/KEY2 */
 uint8_t *sig_b=(uint8_t*)FW_B_APP+b.length-1U;
 *sig_b^=1;
 assert(fw_scan_slots(0,1)==2 && fw_known()==0 && !fw_slots[1].valid);
 uint8_t *body_a=(uint8_t*)FW_APP_ADDR+32;
 *body_a^=1;
 assert(simulate_reset(0,0,0,&trial)==-1); /* neither corrupt image can boot */
 *body_a^=1;*sig_b^=1;
 assert(fw_scan_slots(2,0)==1 && fw_known()==0); /* ECC reset skips all of B */
 *body_a^=1;
 assert(simulate_reset(FW_REVERT_REQ,FW_B_META,2,&trial)==1&&!trial); /* refuse bad fallback */
 *body_a^=1;
 assert(simulate_reset(0,0,0,&trial)==1&&!trial);
 assert(!fw_install(&b,data)); /* cannot overwrite confirmed B */
 memcpy(old_b,(void*)FW_B_META,sizeof(old_b));
 FwManifest a=manifest(argv[2]);assert(fw_install(&a,data));
 assert(!memcmp(old_b,(void*)FW_B_META,sizeof(old_b)));
 assert(simulate_reset(0,0,0,&trial)==0 && trial);
 assert(simulate_reset(FW_CONFIRM_REQ,FW_META_ADDR,3,&trial)==0&&!trial);
 assert(fw_scan_slots(0,1)==1 && fw_slots[0].valid && !fw_slots[1].valid && fw_known()==0);
 assert(simulate_reset(FW_REVERT_REQ,FW_META_ADDR,3,&trial)==1&&!trial);
 printf("A/B: %d torn erase/program/activation/confirmation cases; confirmed slot intact; both update directions and rollback passed\n",count+2);
}
