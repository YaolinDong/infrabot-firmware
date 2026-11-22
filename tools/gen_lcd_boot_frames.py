#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Compile the reference renderer on the host; encode a shared 30 Hz logo for both panel sizes.
No external Python packages. Requires host cc and liblz4. Run after gen_lcd_boot.py.
The MCU expands cropped 4-bit scanlines; the small panel area-filters them.
"""
from pathlib import Path
import tempfile, subprocess, struct, ctypes, ctypes.util
ROOT=Path(__file__).resolve().parents[1]
COUNT=71
# All glyph lighting is a cyan ramp; 16 levels retain edge coverage and bevels.
palette=[]
for i in range(16):
    r=round(45*i/15);g=round(210*i/15);b=round(250*i/15)
    palette.append(((r&248)<<8)|((g&252)<<3)|(b>>3))
def rgb(v):return ((v>>11)*255//31,((v>>5)&63)*255//63,(v&31)*255//31)
colors=[rgb(v) for v in palette]
lut={}
def index(v):
    if v not in lut:
        r,g,b=rgb(v)
        lut[v]=min(range(16),key=lambda i:(r-colors[i][0])**2+(g-colors[i][1])**2+(b-colors[i][2])**2)
    return lut[v]
lz4=ctypes.CDLL(ctypes.util.find_library('lz4'))
lz4.LZ4_compressBound.argtypes=[ctypes.c_int]
ptr=ctypes.c_void_p
integer=ctypes.c_int
lz4.LZ4_createStreamHC.restype=ptr
lz4.LZ4_freeStreamHC.argtypes=[ptr]
lz4.LZ4_resetStreamHC_fast.argtypes=[ptr,integer]
lz4.LZ4_loadDictHC.argtypes=[ptr,ptr,integer]
lz4.LZ4_compress_HC_continue.argtypes=[ptr,ptr,ptr,integer,integer]
lz4.LZ4_decompress_safe_usingDict.argtypes=[ptr,ptr,integer,integer,ptr,integer]

def encode(stream, raw, key):
    lz4.LZ4_resetStreamHC_fast(stream,12)
    if key: assert lz4.LZ4_loadDictHC(stream,key,len(key))==len(key)
    cap=lz4.LZ4_compressBound(len(raw)); dst=ctypes.create_string_buffer(cap)
    size=lz4.LZ4_compress_HC_continue(stream,raw,dst,len(raw),cap)
    assert size>0
    block=dst.raw[:size];check=ctypes.create_string_buffer(len(raw))
    assert lz4.LZ4_decompress_safe_usingDict(block,check,size,len(raw),key,len(key))==len(raw)
    assert check.raw==raw
    return block

def scanlines(values):
    result=bytearray()
    for y in range(280):
        row=values[y*640:(y+1)*640]
        lo=(640-len(row.lstrip(b'\0')))//2
        hi=(len(row.rstrip(b'\0'))+1)//2
        if not hi:lo=0
        result.extend(struct.pack('<HH',lo,hi-lo))
        result.extend((row[2*x]<<4)|row[2*x+1] for x in range(lo,hi))
    return bytes(result)

def checksum(values):
    h=2166136261
    for v in values:h=((h^palette[v])*16777619)&0xffffffff
    return h

def filtered(values):
    # Independent area reference: each dest pixel spans 5/3 source pixels.
    # A subpixel weighs its exact overlap in units of one third of a pixel.
    def weights(i):
        left=i*5;right=left+5
        return [(j,min(right,3*j+3)-max(left,3*j)) for j in range(left//3,(right+2)//3)]
    wx=[weights(x) for x in range(384)]
    for y in range(168):
        wy=weights(y)
        for col in wx:
            yield (12+sum(values[j*640+i]*a*b for j,b in wy for i,a in col))//25

runner=r'''
#include "app_lcd.h"
#include "app_lcd_boot.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
static unsigned width,height;
static uint16_t scratch[800*480];
static FILE *output;
uint16_t AppLcd_PhysicalWidth(void){return width;}
uint16_t AppLcd_PhysicalHeight(void){return height;}
uint16_t *AppLcd_Scratch(void){return scratch;}
void AppLcd_Blit(const uint16_t *p,uint16_t pitch,uint16_t x,uint16_t y,uint16_t w,uint16_t h){
 assert(x+w<=width && y+h<=height);
 if(h<100) return; /* Footer remains native text masks, not baked frames. */
 assert(pitch==w); fwrite(p,2,w*h,output);
}
int main(int argc,char **argv){
 assert(argc==4);width=atoi(argv[1]);height=width==480?272:480;
 output=fopen(argv[2],"wb");assert(output);unsigned count=atoi(argv[3]);
 for(unsigned i=0;i<count;i++)AppLcdBoot_Draw(300U+(2300U*i)/(count-1U));
 fclose(output);return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    tmp=Path(tmp)
    (tmp/'main.h').write_text('#include <stdint.h>\nstatic inline void SCB_CleanDCache_by_Addr(void *p,int32_t n){(void)p;(void)n;}\n')
    (tmp/'runner.c').write_text(runner)
    subprocess.run(['cc','-O2','-I'+str(tmp),'-I'+str(ROOT/'ui'),'-I'+str(ROOT/'drivers/display'),str(tmp/'runner.c'),str(ROOT/'tools/lcd_boot_renderer.c'),'-lm','-o',str(tmp/'render')],check=True)
    raw=tmp/'large.raw'
    subprocess.run([str(tmp/'render'),'800',str(raw),str(COUNT)],check=True)
    data=raw.read_bytes()
    assert len(data)==COUNT*640*280*2
    raws=[];large_hash=[];small_hash=[]
    for i in range(COUNT):
        f=bytes(index(v) for v in struct.unpack_from('<179200H',data,i*640*280*2))
        raws.append(scanlines(f));large_hash.append(checksum(f));small_hash.append(checksum(filtered(f)))
    # Each frame is either independent or references one independent frame.
    # All 71 frames remain directly seekable, including after missed ticks.
    keys=list(range(0,COUNT,4))
    packed=bytearray();offs=[0];refs=[];stream=lz4.LZ4_createStreamHC()
    assert stream
    try:
        for i,source in enumerate(raws):
            block=encode(stream,source,b'');ref=i
            if i not in keys:
                for k in keys:
                    other=encode(stream,source,raws[k])
                    if len(other)<len(block):block,ref=other,k
            refs.append(ref);packed.extend(block);offs.append(len(packed))
    finally:lz4.LZ4_freeStreamHC(stream)
    def array(ctype,name,vals):
        return 'static const '+ctype+' '+name+'[] = {\n'+''.join(
            '  '+','.join(map(str,vals[j:j+24]))+',\n' for j in range(0,len(vals),24))+'};'
    assert max(map(len,raws))<=65535
    out=['/* SPDX-License-Identifier: OFL-1.1 */',
         '/* Generated by tools/gen_lcd_boot_frames.py. Shared 640x280 master, rendered from the\n * Nunito lettering in app_lcd_boot_assets.inc, see LICENSES/OFL-1.1-Nunito.txt. */',
         f'#define BOOT_FRAME_COUNT {COUNT}U',f'#define BOOT_RAW_MAX {max(map(len,raws))}U',
         array('uint16_t','boot_palette',palette),array('uint32_t','boot_offsets',offs),
         array('uint16_t','boot_raw_lengths',list(map(len,raws))),array('uint8_t','boot_refs',refs),
         '#ifdef BOOT_FRAME_TEST_HASHES',array('uint32_t','boot_large_hashes',large_hash),
         array('uint32_t','boot_small_hashes',small_hash),'#endif',
         array('uint8_t','boot_frames',packed)]
    (ROOT/'ui/app_lcd_boot_frames.inc').write_text('\n'.join(out)+'\n')
    print(f'{COUNT} shared frames: {len(packed)} bytes, max unpacked scanlines {max(map(len,raws))} bytes')
