/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "slots.h"
#include <string.h>
FwSlot fw_slots[2];
int fw_active_slot=-1;
static int marked(const FwMark *m,const FwManifest *img,uint32_t type,uint32_t gen)
{
  return m->magic==type && gen && m->generation==gen && m->inverse==~gen &&
    m->base==img->base && !memcmp(m->tag,img->tag,16);
}
static int blank(const uint8_t *p){for(unsigned i=0;i<32;i++)if(p[i]!=255)return 0;return 1;}
static void read_header(unsigned i)
{
  uint32_t base=i?FW_B_META:FW_META_ADDR;
  FwSlot *s=&fw_slots[i];
  const FwManifest *img=(const void*)(uintptr_t)base;
  const FwMark *marks=(const void*)(uintptr_t)(base+64);
  if(!fw_manifest_ok(img)||img->base!=base+1024 ||
     !marked(marks,img,FW_READY_MARK,marks->generation))return;
  s->gen=marks->generation;
  /* A torn attempt/revocation counts as attempted/revoked, never retried. */
  s->tried=!blank((const void*)(marks+1));
  s->good=marked(marks+2,img,FW_GOOD_MARK,s->gen);
  s->revoked=!blank((const void*)(marks+3));
}

static unsigned verify_slot(unsigned i)
{
  FwSlot *s=&fw_slots[i];
  if(!s->gen || s->revoked)return 0;
  const FwManifest *img=(const void*)(uintptr_t)s->base;
  fw_guard(i+1);
  s->valid=fw_boot_image_ok(img,(const uint8_t*)(uintptr_t)img->base);
  fw_guard(0);
  return 1;
}

unsigned fw_scan_slots(unsigned skipped,int ordinary_boot)
{
  for(unsigned i=0;i<2;i++){
    fw_slots[i]=(FwSlot){.base=i?FW_B_META:FW_META_ADDR};
    if(skipped&(1U<<i))continue; /* even a metadata read may fault */
    fw_guard(i+1);read_header(i);fw_guard(0);
  }
  /* Only a normal boot with TWO confirmed, unrevoked slots can omit work.
   * The header ranks candidates; it never authorizes a jump. An unverified
   * slot stays invalid. If the preferred payload is bad, authenticate the
   * fallback before using it. Ties choose A, as in fw_known(). */
  if(ordinary_boot && fw_slots[0].good && !fw_slots[0].revoked &&
                      fw_slots[1].good && !fw_slots[1].revoked){
    unsigned newest=fw_slots[1].gen>fw_slots[0].gen;
    unsigned count=verify_slot(newest);
    return fw_slots[newest].valid?count:count+verify_slot(1-newest);
  }
  unsigned count=verify_slot(0);
  return count+verify_slot(1);
}
int fw_known(void)
{
  int best=-1;
  for(int i=0;i<2;i++)if(fw_slots[i].valid&&fw_slots[i].good &&
    (best<0 || fw_slots[i].gen>fw_slots[best].gen))best=i;
  return best;
}
int fw_candidate(int known)
{
  if(known<0)return -1; /* no previous good app => factory SWD required */
  int j=1-known;FwSlot *s=&fw_slots[j];
  return s->valid && !s->good && !s->tried && s->gen>fw_slots[known].gen?j:-1;
}
uint32_t fw_destination(void)
{
  return fw_active_slot<0?0:fw_slots[1-fw_active_slot].base+1024;
}

/* All state transitions write the candidate/newest slot only. The previous
 * confirmed slot is never modified while testing or confirming an upgrade. */
int fw_select_start(uint32_t reason,uint32_t base,uint32_t gen,int recovery,int *trial)
{
  *trial=0;
  fw_active_slot=fw_known();
  if(fw_active_slot<0)return -1;
  int old=fw_active_slot,other=1-old;FwSlot *s=&fw_slots[other];
  if(reason==FW_CONFIRM_REQ && s->valid && s->tried && !s->good &&
     s->base==base && s->gen==gen && gen>fw_slots[old].gen){
    fw_guard(other+1);
    if(fw_mark(base,128,FW_GOOD_MARK,gen))fw_active_slot=other;
    fw_guard(0);
  } else if(reason==FW_REVERT_REQ && fw_slots[old].base==base && fw_slots[old].gen==gen && s->valid && s->good){
    fw_guard(old+1);
    if(fw_mark(base,160,FW_REVOKE_MARK,gen))fw_active_slot=other;
    fw_guard(0);
  }
  if(reason==FW_REQUEST_MAGIC || recovery)return -1;
  int j=fw_candidate(fw_active_slot);
  if(j>=0){
    s=&fw_slots[j];fw_guard(j+1);
    int marked_ok=fw_mark(s->base,96,FW_TRIED_MARK,s->gen);fw_guard(0);
    if(marked_ok){*trial=1;return j;}
  }
  return fw_active_slot;
}
