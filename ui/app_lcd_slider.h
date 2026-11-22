/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_SLIDER_H
#define APP_LCD_SLIDER_H
#include <stdint.h>
#include <math.h>
#define APP_LCD_SLIDER_NONE UINT8_MAX
typedef struct { uint16_t x,y,width,height; float step; } AppLcdSlider;
typedef struct { uint8_t index; } AppLcdSliderTouch;
/* A gesture belongs to the slider hit on finger-down, until release. */
static inline void AppLcdSlider_Capture(AppLcdSliderTouch *touch,const AppLcdSlider *sliders,
                                       uint8_t count,uint16_t x,uint16_t y)
{
  if (touch->index!=APP_LCD_SLIDER_NONE) return;
  for (uint8_t i=0;i<count;++i) {
    const AppLcdSlider *s=&sliders[i];
    if (x>=s->x && x<=s->x+s->width && y>=s->y && y<s->y+s->height) {
      touch->index=i; return;
    }
  }
}
static inline float AppLcdSlider_Value(const AppLcdSlider *s,uint16_t x,float lo,float hi)
{
  const float fraction=fminf(fmaxf(((float)x-s->x)/s->width,0.f),1.f);
  const float value=lo+roundf(fraction*(hi-lo)/s->step)*s->step;
  return fminf(fmaxf(value,lo),hi);
}
static inline uint16_t AppLcdSlider_Position(const AppLcdSlider *s,float value,float lo,float hi)
{
  return (uint16_t)lroundf(fminf(fmaxf((value-lo)/(hi-lo),0.f),1.f)*s->width);
}
#endif
