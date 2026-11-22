/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd_panel.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void) {
  assert(AppLcdPanel_Detect(480,272)==43);
  assert(AppLcdPanel_Detect(272,480)==43);
  assert(AppLcdPanel_Detect(800,480)==5);
  assert(!AppLcdPanel_Detect(0,0) && !AppLcdPanel_Detect(1024,600));
  assert(AppLcdPanel_ConfigRegister("1158")==0x8051);
  assert(AppLcdPanel_ConfigRegister("5688")==0x8051);
  assert(AppLcdPanel_ConfigRegister("911")==0x8048);
  assert(!AppLcdPanel_ConfigRegister("????"));
  const AppLcdPanel *p=AppLcdPanel_Get(43,0);
  assert(p->width==480 && p->height==272);
  assert(p->hsw+p->hbp+p->width+p->hfp==531);
  assert(p->vsw+p->vbp+p->height+p->vfp==292);
  assert(5*p->pll_n/p->pll_r==9);
  p=AppLcdPanel_Get(5,0);assert(5*p->pll_n/p->pll_r==27);
  uint16_t lx,ly;
  assert(!AppLcdPanel_Touch(480,0,480,272,43,&lx,&ly));
  assert(!AppLcdPanel_Touch(0,0,800,480,43,&lx,&ly));
  assert(!AppLcdPanel_Touch(0,0,0,0,43,&lx,&ly));
  assert(AppLcdPanel_Touch(240,136,480,272,43,&lx,&ly) && lx==240 && ly==136);
  assert(AppLcdPanel_Touch(136,240,272,480,43,&lx,&ly) && lx==240 && ly==136);
  for(unsigned mode=0;mode<2;++mode) {
    unsigned pw=mode?800:480,ph=mode?480:272;
    for(unsigned yy=0;yy<ph;++yy) for(unsigned xx=0;xx<pw;++xx) {
      assert(AppLcdPanel_Touch(xx,yy,pw,ph,mode?5:43,&lx,&ly));
      assert(lx==xx && ly==yy);
    }
  }
  puts("LCD panels: profiles, ID maps, touch ranges, rotation, native pixel coordinates passed");
}
