/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Verify every generated frame against the generator's uncompressed hash,
 * both native layouts, scratch guards, and bounded malformed-block handling. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define BOOT_FRAME_TEST_HASHES
#include "app_lcd_boot.c"
static uint16_t storage[APP_LCD_SCRATCH_PIXELS+32] __attribute__((aligned(32)));
static unsigned screen_width,screen_height,blits;
static unsigned logo_bottom,footer_bottom,visible_links,logo_ink,footer_ink;
uint16_t AppLcd_PhysicalWidth(void){return screen_width;}
uint16_t AppLcd_PhysicalHeight(void){return screen_height;}
uint16_t *AppLcd_Scratch(void){return storage;}
void SCB_CleanDCache_by_Addr(void *p,int32_t bytes){
  assert(p==storage && bytes>0 && bytes%32==0);
  assert((unsigned)bytes<=APP_LCD_SCRATCH_PIXELS*2U);
}
void AppLcd_Blit(const uint16_t *p,uint16_t pitch,uint16_t x,uint16_t y,uint16_t w,uint16_t h){
  assert(p==storage && pitch==w);
  assert(x+w<=screen_width && y+h<=screen_height);++blits;
  unsigned ink=0;
  for(unsigned i=0;i<(unsigned)w*h;i++)ink+=(p[i]!=0U);
  if(h==SMALL_H || h==MASTER_H) {logo_bottom=y+h;logo_ink=ink;}
  else {assert(y>=logo_bottom);footer_bottom=y+h;footer_ink=ink;}
}
void AppLcd_DrawText(uint16_t x,uint16_t y,const char *s,uint8_t size,uint16_t fg,uint16_t bg){
  assert(strcmp(s,"https://github.com/YaolinDong")==0);
  assert(size==(screen_width==480?16:24));
  assert(2U*x+strlen(s)*(size/2U)==screen_width);
  assert(footer_bottom<=y && y+size+8U==screen_height && bg==0);
  if(fg) ++visible_links;
}
static void CheckFrames(int small,const uint32_t *hashes,unsigned pixels){
  /* Seek out of order; never rely on the previous displayed frame. */
  for(unsigned i=0;i<BOOT_FRAME_COUNT;i++){
    unsigned f=(37U*i)%BOOT_FRAME_COUNT;
    assert(boot_refs[boot_refs[f]]==boot_refs[f] && boot_raw_lengths[f]<=BOOT_RAW_MAX);
    storage[APP_LCD_SCRATCH_PIXELS]=0xabcd;
    assert(Logo(storage,f,small,boot_palette));
    uint32_t hash=2166136261U;
    for(unsigned j=0;j<pixels;j++) hash=(hash^storage[j])*16777619U;
    assert(hash==hashes[f] && storage[APP_LCD_SCRATCH_PIXELS]==0xabcd);
  }
}
int main(void){
  CheckFrames(0,boot_large_hashes,MASTER_W*MASTER_H);
  CheckFrames(1,boot_small_hashes,SMALL_W*SMALL_H);
  for(unsigned model=0;model<2;model++){
    screen_width=model?800:480;screen_height=model?480:272;
    storage[APP_LCD_SCRATCH_PIXELS]=0xabcd;
    visible_links=0;logo_ink=footer_ink=0;
    AppLcdBoot_Draw(0U);
    assert(logo_ink>1000 && footer_ink>100 && visible_links==1);
    for(unsigned ms=25;ms<=3200;ms+=25) AppLcdBoot_Draw(ms);
    assert(visible_links && storage[APP_LCD_SCRATCH_PIXELS]==0xabcd);
  }
  uint8_t dst[17],bad[][4]={{0xf0,255,255,255},{0,0,0,0},{0x10,9,2,0}};
  for(unsigned i=0;i<3;i++){
    memset(dst,0xaa,sizeof(dst));
    assert(!Decode(dst,16,bad[i],bad[i]+sizeof(bad[i]),NULL,0) && dst[16]==0xaa);
  }
  uint8_t overflow_row[]={0x40,1,1,0,0xff}; /* first pair = 320, no room */
  assert(!Raster(storage,0,boot_palette,overflow_row,sizeof(overflow_row),(uint8_t *)(storage+MASTER_W*MASTER_H)));
  assert(blits>0);
  puts("Boot animation: 71 shared frames verified at native and area-filtered sizes; seeking, bounds and first light passed");
}
