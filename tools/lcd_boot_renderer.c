/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd_boot.h"
#include "app_lcd.h"
#include "main.h"
#include <math.h>

typedef struct { uint16_t width, height; const uint8_t *data, *bevel; } BootMask;
#include "app_lcd_boot_assets.inc"

static float Coverage(const BootMask *m, int x, int y)
{
  if (x<0 || y<0 || x>=m->width || y>=m->height) return 0.0f;
  unsigned i=(unsigned)y*m->width+(unsigned)x;
  return (float)((m->data[i/2] >> ((i&1U)?0:4))&15U);
}
/* Alpha blending preserves coverage on both the front face and extruded edges. */
static uint16_t Blend(uint16_t bg, float coverage, float light, float fade)
{
  unsigned alpha=(unsigned)(coverage*fade*17.0f+0.5f);
  if(alpha>255U) alpha=255U;
  unsigned r=(unsigned)(45.0f*light),g=(unsigned)(210.0f*light),b=(unsigned)(250.0f*light);
  if(r>255U) r=255U;
  if(g>255U) g=255U;
  if(b>255U) b=255U;
  r=(r*alpha+((bg>>11)*255U/31U)*(255U-alpha))/255U;
  g=(g*alpha+(((bg>>5)&63U)*255U/63U)*(255U-alpha))/255U;
  b=(b*alpha+((bg&31U)*255U/31U)*(255U-alpha))/255U;
  return APP_LCD_RGB565(r,g,b);
}
static float Sample(const BootMask *m, float u, float v)
{
  int ix=(int)floorf(u),iy=(int)floorf(v);
  float fx=u-ix,fy=v-iy;
  float a=Coverage(m,ix,iy)*(1.0f-fx)+Coverage(m,ix+1,iy)*fx;
  float b=Coverage(m,ix,iy+1)*(1.0f-fx)+Coverage(m,ix+1,iy+1)*fx;
  return a*(1.0f-fy)+b*fy;
}
/* Perspective projection of an extruded glyph, sliced back-to-front along
 * local Z. Rotation is about the vertical screen Y axis: the word turns left/right.
 * Unlike an affine X scale, this retains perspective and visible sidewalls
 * even as the front face becomes edge-on. Only the logo rectangle is redrawn. */
static void DrawLogo(uint16_t *buf,unsigned width,unsigned height,
                     const BootMask *logo,float angle,float fade)
{
  const float c=cosf(angle),s=sinf(angle);
  const float distance=logo->width*1.5f;
  const float depth=logo->height*0.18f;
  const int slices=(int)ceilf(depth*2.0f);
  const float cx=(width-1U)*0.5f,cy=(height-1U)*0.5f;
  for(unsigned i=0;i<width*height;i++) buf[i]=0;
  /* A soft blue ground shadow anchors the floating word. */
  for(unsigned y=0;y<height;y++) {
    float dy=((float)y-height*0.85f)/(height*0.055f);
    if(fabsf(dy)>=1.0f) continue;
    for(unsigned x=0;x<width;x++) {
      float dx=((float)x-cx)/(logo->width*0.48f);
      float falloff=1.0f-dx*dx-dy*dy;
      if(falloff>0.0f) {
        unsigned a=(unsigned)(falloff*falloff*fade*32.0f);
        buf[y*width+x]=APP_LCD_RGB565(0U,a/2U,a);
      }
    }
  }
  for(int layer=0;layer<=slices;layer++) {
    float z=(-depth+2.0f*depth*layer/slices)*(c>=0.0f?1.0f:-1.0f);
    const uint8_t face=layer==slices;
    /* Rounded shoulder over the final ~3 pixels of each end cap. */
    float shoulder=fmaxf(0.0f,(fabsf(z)-depth+3.0f)/3.0f);
    float bevel=1.0f-sqrtf(fmaxf(0.0f,1.0f-shoulder*shoulder));
    BootMask inset=*logo;
    inset.data=logo->bevel;
    float side_light=0.48f+0.22f*fabsf(s);
    float face_light=0.72f+0.28f*fabsf(c);
    float light=side_light+(face_light-side_light)*shoulder;
    for(unsigned x=0;x<width;x++) {
      float dx=(float)x-cx;
      float denominator=distance*c+dx*s;
      if(fabsf(denominator)<0.0001f) continue;
      float local_x=(dx*(distance-c*z)+distance*s*z)/denominator;
      float u=local_x+(logo->width-1U)*0.5f;
      if(u < -1.0f || u>logo->width) continue;
      float inv_scale=(distance-s*local_x-c*z)/distance;
      if(inv_scale<=0.0f) continue;
      int top=(int)floorf(cy-(logo->height*0.5f+1.0f)/inv_scale);
      int bottom=(int)ceilf(cy+(logo->height*0.5f+1.0f)/inv_scale);
      if(top<0) top=0;
      if(bottom>(int)height) bottom=(int)height;
      for(int y=top;y<bottom;y++) {
        float v=((float)y-cy)*inv_scale+(logo->height-1U)*0.5f;
        float row_light=light+(face?0.08f*(1.0f-v/logo->height):0.0f);
        float coverage=Sample(logo,u,v);
        if(bevel>0.001f && logo->bevel)
          coverage+=(Sample(&inset,u,v)-coverage)*bevel;
        if(coverage>0.0f) buf[(unsigned)y*width+x]=Blend(buf[(unsigned)y*width+x],coverage,row_light,fade);
      }
    }
  }
}
static void Present(uint16_t *buf, unsigned w, unsigned h, unsigned x, unsigned y)
{
  /* Scratch is cacheable SDRAM, aligned to 1 MB; CPU overwrites every pixel.
   * Publish to DMA2D only after cleaning complete enclosing cache lines. */
  SCB_CleanDCache_by_Addr((void *)buf, (int32_t)((w*h*2U+31U)&~31U));
  AppLcd_Blit(buf,(uint16_t)w,(uint16_t)x,(uint16_t)y,(uint16_t)w,(uint16_t)h);
}
void AppLcdBoot_Draw(uint32_t elapsed_ms)
{
  const unsigned w=AppLcd_PhysicalWidth(), h=AppLcd_PhysicalHeight();
  const uint8_t small=w<800U;
  const BootMask *logo=small?&boot_logo_small:&boot_logo_large;
  const BootMask *footer=small?&boot_footer_small:&boot_footer_large;
  const unsigned tile_w=small?384U:640U,tile_h=small?164U:280U;
  uint16_t *buf=AppLcd_Scratch();
  float fade=elapsed_ms<300U ? elapsed_ms/300.0f : 1.0f;
  float t=elapsed_ms<300U?0.0f:(elapsed_ms>=2600U?1.0f:(elapsed_ms-300U)/2300.0f);
  /* Quintic easing gives zero velocity and acceleration at both ends. */
  float eased=t*t*t*(10.0f+t*(-15.0f+6.0f*t));
  DrawLogo(buf,tile_w,tile_h,logo,-6.28318530718f*eased,fade);
  Present(buf,tile_w,tile_h,(w-tile_w)/2U,small?24U:54U);
  for (unsigned y=0;y<footer->height;y++)
    for (unsigned x=0;x<footer->width;x++) {
      unsigned a=(unsigned)(Coverage(footer,(int)x,(int)y)*fade*14.0f);
      buf[y*footer->width+x]=APP_LCD_RGB565(a,a,a);
    }
  Present(buf,footer->width,footer->height,(w-footer->width)/2U,h-footer->height-8U);
}
