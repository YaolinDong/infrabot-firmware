/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "update.h"
#include "consent.h"
#include "app_lcd_update_glyphs.inc"

/* Entirely in the low-priority LCD task. No motor actions behind the modal. */
static void UpdateButton(unsigned x,unsigned y,unsigned w,int allow,int rollback){
  const uint16_t bg=allow?APP_LCD_RGB565(18,110,62):APP_LCD_RGB565(153,44,42);
  AppLcd_FillRect(x,y,w,52,bg);
  unsigned letters[]={allow?0U:2U,allow?1U:3U,rollback?6U:4U,rollback?7U:5U};
  uint16_t *buf=AppLcd_Scratch();
  for(unsigned j=0;j<24;j++)for(unsigned i=0;i<96;i++)
    buf[j*96+i]=fw_glyphs[letters[i/24]][j*3+(i%24)/8]&(128U>>(i%8))?UI_TEXT:bg;
  SCB_CleanDCache_by_Addr((void*)buf,96*24*2);
  AppLcd_Blit(buf,96,x+(w-96)/2,y+14,96,24);
}
static int UpdateModal(uint8_t was_touched,uint8_t *redraw){
  static uint64_t displayed;
  static uint8_t await_lift;
  FwPrompt p;
  if(!fw_consent_view(&p)){
    if(displayed){displayed=0;*redraw=1;ui_slots[UI_SLOT_STATUS].valid=0;}
    if(await_lift && ui_touch.touched)return 1;
    await_lift=0;return 0;
  }
  const unsigned w=AppLcd_PhysicalWidth(),h=AppLcd_PhysicalHeight(),x=w/2-168,y=h/2+20,bw=156;
  if(displayed!=p.id){
    displayed=p.id;await_lift=ui_touch.touched;
    ui_pressed=UI_NO_BUTTON;ui_slider.index=ui_imp_slider.index=APP_LCD_SLIDER_NONE;ui_touch.dragging=0;
    AppLcd_FillRect(0,0,w,h,UI_BG);
    char msg[64];
    AppLcd_DrawText(x,y-100,p.op==FW_REVERT?"Rollback firmware?":"Install firmware?",24,UI_TEXT,UI_BG);
    const uint8_t *ip=(const uint8_t*)&p.ip;
    snprintf(msg,sizeof(msg),"From %u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
    AppLcd_DrawText(x,y-64,msg,16,UI_TEXT,UI_BG);
    if(p.op==FW_ENTER){
      snprintf(msg,sizeof(msg),"Image %02x%02x%02x%02x",p.tag[0],p.tag[1],p.tag[2],p.tag[3]);
      AppLcd_DrawText(x,y-40,msg,16,UI_TEXT,UI_BG);
    }
    UpdateButton(x,y,bw,0,p.op==FW_REVERT);UpdateButton(x+180,y,bw,1,p.op==FW_REVERT);
    return 1;
  }
  if(!ui_touch.touched)await_lift=0;
  if(!await_lift && !was_touched && ui_touch.touched && ui_touch.points==1 && ui_touch.y>=y && ui_touch.y<y+52){
    unsigned tx=ui_touch.x;
    if((tx>=x && tx<x+bw)||(tx>=x+180 && tx<x+180+bw)){
      await_lift=1;fw_consent_decide(displayed,tx>=x+180);
    }
  }
  return 1;
}
