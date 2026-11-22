/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"
#include "slots.h"
#include <assert.h>
#include <sys/mman.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
volatile uint32_t host_mail[24];
static MockFlash regs; MockFlash *FLASH=&regs;
uint16_t flash_kb=2048;
int cut_at,operations,locked=1;
jmp_buf cut_env;
int HAL_FLASH_Unlock(void){locked=0;return HAL_OK;}
void HAL_FLASH_Lock(void){locked=1;}
void fw_guard(unsigned slot){(void)slot;}
void map_flash(void){
 assert(mmap((void*)FLASH_BASE,0x200000,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0)!=(void*)-1);
}
void load_factory(const char *file){
 memset((void*)FLASH_BASE,255,0x200000);
 memset((void*)FW_CFG_ADDR,0xae,0x20000);
 FILE *f=fopen(file,"rb");assert(f);assert(fread((void*)FLASH_BASE,1,0x100000,f)>0x20400);fclose(f);
 operations=0;cut_at=0;fw_scan_slots(0,1);fw_active_slot=fw_known();assert(fw_active_slot==0);
}
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *error){
 (void)error;assert(!locked&&e->NbSectors==1);unsigned n=(e->Banks-1)*8+e->Sector;
 assert(n>=1&&n<=14 && fw_active_slot>=0 && (n<8)!=(fw_active_slot==0));
 unsigned char *p=(void*)(uintptr_t)(FLASH_BASE+n*0x20000U);memset(p,255,0x10000);
 if(++operations==cut_at)longjmp(cut_env,1);
 memset(p+0x10000,255,0x10000);return 0;
}
int HAL_FLASH_Program(int type,uint32_t address,uintptr_t source){
 (void)type;assert(!locked && address>=FW_META_ADDR && address+32<=FW_CFG_ADDR && address%32==0);
 unsigned char *dst=(void*)(uintptr_t)address;const unsigned char *src=(void*)source;
 for(unsigned i=0;i<32;i++){assert(dst[i]==255);dst[i]=src[i];if(i==15&&++operations==cut_at)longjmp(cut_env,1);}
 return 0;
}
int simulate_reset(uint32_t why,uint32_t meta,uint32_t gen,int *trial){
 locked=1;fw_scan_slots(0,!why);
 for(unsigned i=0;i<0x20000;i++)assert(((const uint8_t*)FW_CFG_ADDR)[i]==0xae);
 return fw_select_start(why,meta,gen,0,trial);
}
