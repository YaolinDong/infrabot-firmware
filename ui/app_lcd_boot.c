/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd_boot.h"
#include "app_lcd.h"
#include "main.h"
#include <string.h>

typedef struct { uint16_t width, height; const uint8_t *data, *bevel; } BootMask;
#include "app_lcd_boot_assets.inc"

static float Coverage(const BootMask *m, int x, int y)
{
  if (x<0 || y<0 || x>=m->width || y>=m->height) return 0.0f;
  unsigned i=(unsigned)y*m->width+(unsigned)x;
  return (float)((m->data[i/2] >> ((i&1U)?0:4))&15U);
}
#include "app_lcd_boot_frames.inc"

/* One shared master. The scratch layout is output | key | block | row workspace.
 * The largest output and all temporary storage fit inside existing cached SDRAM. */
enum { MASTER_W=640, MASTER_H=280, SMALL_W=384, SMALL_H=168 };
_Static_assert(MASTER_W*3==SMALL_W*5 && MASTER_H*3==SMALL_H*5,"5:3 area filter");
_Static_assert(MASTER_W*MASTER_H*2 + 2*BOOT_RAW_MAX + MASTER_W + 5*SMALL_W
               <= APP_LCD_SCRATCH_PIXELS*2U,"boot decoder scratch");

static unsigned Length(const uint8_t **src,const uint8_t *end,unsigned n)
{
  if(n==15U){
    unsigned byte;
    do{
      if(*src==end)return UINT32_MAX;
      byte=*(*src)++;n+=byte;
    }while(byte==255U);
  }
  return n;
}

/* Standard LZ4 block with an optional external dictionary. Never read before
 * either buffer, including when a match crosses from the key into this block. */
static int Decode(uint8_t *dst,unsigned size,const uint8_t *src,const uint8_t *end,
                  const uint8_t *key,unsigned key_size)
{
  unsigned w=0;
  while(src<end){
    unsigned token=*src++,n=Length(&src,end,token>>4);
    if(n>size-w || n>(unsigned)(end-src))return 0;
    memcpy(dst+w,src,n);w+=n;src+=n;
    if(src==end)return w==size;
    if(end-src<2)return 0;
    unsigned off=src[0]|((unsigned)src[1]<<8);src+=2;
    if(!off || off>w+key_size)return 0;
    n=Length(&src,end,token&15U);
    if(n>size-w || size-w-n<4U)return 0;
    n+=4U;
    if(off>w){
      unsigned back=off-w,take=n<back?n:back;
      memcpy(dst+w,key+key_size-back,take);w+=take;n-=take;
    }
    while(n--){dst[w]=dst[w-off];++w;}
  }
  return 0;
}

/* Each trimmed row: little-endian pair offset, pair count, then two palette
 * indices per byte. Metadata validated before touching the output row. */
static int Row(const uint8_t **src,const uint8_t *end,unsigned *offset,unsigned *count)
{
  const uint8_t *p=*src;
  if(end-p<4)return 0;
  unsigned x=p[0]|((unsigned)p[1]<<8),n=p[2]|((unsigned)p[3]<<8);
  if(x>MASTER_W/2 || n>MASTER_W/2-x || n>(unsigned)(end-p-4))return 0;
  *offset=x*2;*count=n;*src=p+4;return 1;
}

static int Raster(uint16_t *dst,int small,const uint16_t palette[16],
                  const uint8_t *p,unsigned len,uint8_t *row)
{
  const uint8_t *end=p+len;
  uint8_t *hrows=row+MASTER_W; /* five horizontal weighted sums, each <=75 */
  unsigned first=SMALL_W/3,last=0;
  for(unsigned y=0;y<MASTER_H;y++){
    unsigned x,n;
    if(!Row(&p,end,&x,&n))return 0;
    if(!small){
      memset(dst,0,MASTER_W*sizeof(*dst));
      for(unsigned j=0;j<n;j++){unsigned b=*p++;dst[x++]=palette[b>>4];dst[x++]=palette[b&15U];}
      dst+=MASTER_W;
    }else{
      memset(row,0,MASTER_W);
      unsigned begin=x/5U,stop=(x+2U*n+4U)/5U;
      if(n && begin<first)first=begin;
      if(n && stop>last)last=stop;
      for(unsigned j=0;j<n;j++){unsigned b=*p++;row[x++]=b>>4;row[x++]=b&15U;}
      uint8_t *h=hrows+(y%5U)*SMALL_W;
      memset(h,0,SMALL_W);
      for(unsigned g=begin;g<stop;g++){
        const uint8_t *v=row+5U*g;
        h[3*g]=3*v[0]+2*v[1];h[3*g+1]=v[1]+3*v[2]+v[3];h[3*g+2]=2*v[3]+3*v[4];
      }
      if(y%5U!=4U)continue;
      /* Don't filter black margins. The complete rectangle is still
       * published, clearing the silhouette of the preceding frame. */
      memset(dst,0,3*SMALL_W*sizeof(*dst));
      for(unsigned j=3*first;j<3*last;j++){
        const uint8_t *v=hrows+j;
        dst[j]=palette[(3*v[0]+2*v[SMALL_W]+12)/25];
        dst[SMALL_W+j]=palette[(v[SMALL_W]+3*v[2*SMALL_W]+v[3*SMALL_W]+12)/25];
        dst[2*SMALL_W+j]=palette[(2*v[3*SMALL_W]+3*v[4*SMALL_W]+12)/25];
      }
      dst+=3*SMALL_W;
      first=SMALL_W/3;last=0;
    }
  }
  return p==end;
}

static int Logo(uint16_t *buf,unsigned frame,int small,const uint16_t palette[16])
{
  uint8_t *key=(uint8_t *)(buf+MASTER_W*MASTER_H),*block=key+BOOT_RAW_MAX;
  unsigned ref=boot_refs[frame],key_size=0;
  if(ref!=frame){
    key_size=boot_raw_lengths[ref];
    if(!Decode(key,key_size,boot_frames+boot_offsets[ref],boot_frames+boot_offsets[ref+1],NULL,0))return 0;
  }
  unsigned size=boot_raw_lengths[frame];
  return Decode(block,size,boot_frames+boot_offsets[frame],boot_frames+boot_offsets[frame+1],key,key_size)
      && Raster(buf,small,palette,block,size,block+BOOT_RAW_MAX);
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
  static const char github_url[]="https://github.com/YaolinDong";
  static unsigned finished_footer_width;
  const unsigned w=AppLcd_PhysicalWidth(), h=AppLcd_PhysicalHeight();
  const uint8_t small=w<800U;
  const BootMask *footer=small?&boot_footer_small:&boot_footer_large;
  const unsigned tile_w=small?SMALL_W:MASTER_W,tile_h=small?SMALL_H:MASTER_H;
  uint16_t *buf=AppLcd_Scratch();
  /* Frame zero is also the splash shown before the scheduler starts. */
  float fade=elapsed_ms<300U ? 0.65f+0.35f*(elapsed_ms/300.0f) : 1.0f;
  unsigned frame=elapsed_ms<=300U?0U:
    (elapsed_ms>=2600U?BOOT_FRAME_COUNT-1U:(elapsed_ms-300U)*(BOOT_FRAME_COUNT-1U)/2300U);
  uint16_t palette[16];
  for(unsigned i=0;i<16U;i++) {
    uint16_t color=boot_palette[i];
    unsigned r=(unsigned)((color>>11)*fade),g=(unsigned)(((color>>5)&63U)*fade);
    unsigned b=(unsigned)((color&31U)*fade);
    palette[i]=(uint16_t)((r<<11)|(g<<5)|b);
  }
  if(Logo(buf,frame,small,palette))
    Present(buf,tile_w,tile_h,(w-tile_w)/2U,small?22U:54U);
  /* The logo never overlaps the credits. Leave them on screen once fully lit. */
  if (elapsed_ms>=300U && finished_footer_width==w) return;
  finished_footer_width=elapsed_ms>=300U?w:0U;
  const unsigned font_size=small?16U:24U;
  const unsigned link_y=h-8U-font_size;
  const unsigned link_width=(sizeof(github_url)-1U)*(font_size/2U);
  for (unsigned y=0;y<footer->height;y++)
    for (unsigned x=0;x<footer->width;x++) {
      unsigned a=(unsigned)(Coverage(footer,(int)x,(int)y)*fade*14.0f);
      buf[y*footer->width+x]=APP_LCD_RGB565(a,a,a);
    }
  Present(buf,footer->width,footer->height,(w-footer->width)/2U,link_y-4U-footer->height);
  unsigned shade=(unsigned)(210.0f*fade);
  AppLcd_DrawText((uint16_t)((w-link_width)/2U),(uint16_t)link_y,github_url,
                  (uint8_t)font_size,APP_LCD_RGB565(shade,shade,shade),0U);
}
