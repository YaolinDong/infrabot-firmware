/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd_slider.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
  const AppLcdSlider sliders[]={{40,202,480,48,.05f},{40,254,480,48,.5f}};
  AppLcdSliderTouch touch={APP_LCD_SLIDER_NONE};
  AppLcdSlider_Capture(&touch,sliders,2,100,180); assert(touch.index==APP_LCD_SLIDER_NONE);
  AppLcdSlider_Capture(&touch,sliders,2,280,222); assert(touch.index==0);
  assert(AppLcdSlider_Value(&sliders[0],0,0,3)==0);
  assert(AppLcdSlider_Value(&sliders[0],799,0,3)==3);
  assert(fabsf(AppLcdSlider_Value(&sliders[0],280,0,3)-1.5f)<1e-6);
  AppLcdSlider_Capture(&touch,sliders,2,100,280); assert(touch.index==0); /* still captured */
  touch.index=APP_LCD_SLIDER_NONE;
  AppLcdSlider_Capture(&touch,sliders,2,100,280); assert(touch.index==1);
  for(unsigned x=0;x<800;++x) {
    float v=AppLcdSlider_Value(&sliders[1],x,.5f,30.f);
    assert(v>=.5f && v<=30.f && fabsf(v*2-roundf(v*2))<1e-6);
    assert(AppLcdSlider_Position(&sliders[1],v,.5f,30.f)<=480);
  }
  puts("slider hit/capture, edge clamps, quantization: OK");
}
