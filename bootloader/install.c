/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"
#include "slots.h"
#include <string.h>

static HAL_StatusTypeDef word(uint32_t address,const void *data,unsigned n)
{
  uint32_t aligned[8] __attribute__((aligned(32)));
  memset(aligned,0xff,sizeof(aligned));memcpy(aligned,data,n);
  return HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,address,(uintptr_t)aligned);
}
/* Markers each occupy a distinct flashword: no H7 ECC word is programmed twice. */
int fw_mark(uint32_t base,unsigned offset,uint32_t type,uint32_t generation)
{
  if((base!=FW_META_ADDR && base!=FW_B_META) || (FLASH->OPTSR_CUR&FLASH_OPTSR_SWAP_BANK_OPT) ||
     !((offset==64 && type==FW_READY_MARK)||(offset==96 && type==FW_TRIED_MARK)||
       (offset==128 && type==FW_GOOD_MARK)||(offset==160 && type==FW_REVOKE_MARK)))return 0;
  const uint8_t *dest=(const void*)(uintptr_t)(base+offset);
  for(unsigned k=0;k<32;k++)if(dest[k]!=255)return 0;
  FwMark m={type,generation,~generation,base+1024,{0}};
  memcpy(m.tag,(const void*)(uintptr_t)(base+32),16);
  if(HAL_FLASH_Unlock()!=HAL_OK)return 0;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);
  int ok=word(base+offset,&m,32)==HAL_OK && !memcmp(dest,&m,32);
  HAL_FLASH_Lock();return ok;
}
int fw_install(const FwManifest *m,const uint8_t *stage)
{
  if(fw_active_slot<0 || fw_slots[fw_active_slot].gen==UINT32_MAX || m->base!=fw_destination() || !fw_image_ok(m,stage) || *(volatile uint16_t *)FLASHSIZE_BASE!=2048U ||
     (FLASH->OPTSR_CUR&FLASH_OPTSR_SWAP_BANK_OPT) || HAL_FLASH_Unlock()!=HAL_OK)return 0;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);
  int ok=0;
  unsigned first=(m->base-1024U-FLASH_BASE)/0x20000U;
  unsigned last=(m->base+m->length-1U-FLASH_BASE)/0x20000U;
  /* Erase obsolete manifest FIRST. Sector 0 (loader) and sector 15
   * (calibration/network) are outside the permitted interval. */
  if((first!=1 && first!=8)||last>first+6)goto done;
  for(unsigned i=first;i<=last;i++){
    FLASH_EraseInitTypeDef erase={.TypeErase=FLASH_TYPEERASE_SECTORS,
      .Banks=i<8?FLASH_BANK_1:FLASH_BANK_2,.Sector=i%8,.NbSectors=1,.VoltageRange=FLASH_VOLTAGE_RANGE_3};
    uint32_t error;
    if(HAL_FLASHEx_Erase(&erase,&error)!=HAL_OK)goto done;
    fw_progress();
  }
  for(unsigned i=0;i<m->length;i+=32){
    if(word(m->base+i,stage+i,m->length-i<32?m->length-i:32)!=HAL_OK)goto done;
    if((i&4095U)==0)fw_progress();
  }
  /* Loader never enables D-cache. Verify from actual Flash before committing
   * metadata. The READY marker is programmed only after manifest readback. */
  if(!fw_image_ok(m,(const uint8_t *)(uintptr_t)m->base))goto done;
  if(word(m->base-1024+32,m->tag,32)!=HAL_OK || word(m->base-1024,m,32)!=HAL_OK)goto done;
  ok=!memcmp((const void *)(uintptr_t)(m->base-1024),m,sizeof(*m));
done:
  HAL_FLASH_Lock();
  return ok && fw_mark(m->base-1024,64,FW_READY_MARK,fw_slots[fw_active_slot].gen+1U);
}
