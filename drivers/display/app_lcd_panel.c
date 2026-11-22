/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd_panel.h"
#include <string.h>
/* GL043056B0-40-N page 11: 9MHz, total 531x292. The specified back
 * blanking (43/12) includes the sync pulses (4/4): HAL porches are 39/8. */
static const AppLcdPanel panel43 = {480,272,4,39,8,4,8,8,54,30,9,43};
/* Older compatible example's INCH_4_3 entry, 533x288 at 9MHz. */
static const AppLcdPanel panel43_legacy = {480,272,41,8,4,10,2,4,54,30,9,43};
static const AppLcdPanel panel5 = {800,480,1,46,40,3,23,13,54,10,27,5};
const AppLcdPanel *AppLcdPanel_Get(uint8_t model, uint8_t legacy43) {
  return model == 43U ? (legacy43 ? &panel43_legacy : &panel43) : &panel5;
}
uint8_t AppLcdPanel_Detect(uint16_t x, uint16_t y) {
  if ((x==480U && y==272U) || (x==272U && y==480U)) return 43U;
  if ((x==800U && y==480U) || (x==480U && y==800U)) return 5U;
  return 0U;
}
uint16_t AppLcdPanel_ConfigRegister(const char *id) {
  if (!strncmp(id,"115",3) || !strncmp(id,"5688",4) || !strncmp(id,"917S",4)) return 0x8051U;
  if (!strncmp(id,"911",3) || !strncmp(id,"9157",4) || !strncmp(id,"615",3)) return 0x8048U;
  return 0U;
}
uint8_t AppLcdPanel_Touch(uint16_t x, uint16_t y, uint16_t rx, uint16_t ry,
                          uint8_t model, uint16_t *lx, uint16_t *ly) {
  if (!rx || !ry || x>=rx || y>=ry || AppLcdPanel_Detect(rx,ry)!=model) return 0U;
  if (rx < ry) { uint16_t t=x; x=y; y=t; t=rx; rx=ry; ry=t; }
  *lx=x;
  *ly=y;
  return 1U;
}
