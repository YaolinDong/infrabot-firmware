/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_LCD_3D_H
#define APP_LCD_3D_H

#include <stdint.h>

#ifndef APP_LCD_PARTIAL_REFRESH
#define APP_LCD_PARTIAL_REFRESH 1
#endif
#ifndef APP_LCD_FAST_BUFFER
#define APP_LCD_FAST_BUFFER 1
#endif

/* Live 3-D model of the arm (digital twin) for the LCD's 3D page: the
 * decimated URDF meshes (app_lcd_mesh.c) posed by the measured joints,
 * flat-shaded, both sides drawn, painter-sorted, plus a floor grid and the
 * tool axes.  Orthographic orbit
 * camera with the PC GUI viewer's angles and home view.  Drawn off screen in
 * AXI SRAM tiles of at most 65536 pixels, each submitted by DMA2D.  Disabling
 * APP_LCD_FAST_BUFFER restores SDRAM drawing.  Fixed-camera updates cover
 * the old/new model bounds; camera changes also include the grid/base.
 * Page/viewport changes repaint the full view.  An unchanged frame is
 * skipped.  LCD task only. */

typedef enum
{
  APP_LCD3D_VIEW_HOME,   /* the PC GUI viewer's home view */
  APP_LCD3D_VIEW_FRONT,  /* looking along -X */
  APP_LCD3D_VIEW_SIDE,   /* looking along +Y */
  APP_LCD3D_VIEW_TOP
} AppLcd3dView;

void AppLcd3d_SetView(AppLcd3dView view);
/* Turns the camera by a finger drag of (dx, dy) pixels. */
void AppLcd3d_Orbit(int16_t dx, int16_t dy);

/* Renders the current arm state into the screen rectangle (x, y, w, h),
 * w * h <= APP_LCD_SCRATCH_PIXELS.  Returns 1 when it drew, 0 when nothing changed since
 * the last frame (the screen still shows it). */
uint8_t AppLcd3d_Render(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
typedef struct
{
  uint32_t prepare_cycles;
  uint32_t draw_cycles;
  uint32_t clear_cycles;  /* CPU clear, or DMA clear including invalidation */
  uint32_t raster_cycles; /* grid, triangles and tool axes */
  uint32_t clean_cycles;  /* publish CPU writes for DMA2D */
  uint32_t blit_cycles;   /* DMA2D copy to the display */
  uint32_t updated_pixels; /* area cleared and copied in this frame */
  uint32_t sram_buffer;   /* 1: all tiles in AXI SRAM; 0: SDRAM scratch */
  uint32_t tiles;         /* completed DMA2D rectangle submissions */
} AppLcd3dTiming;

/* CPU cycles of the last rendered frame, including time preempted by
 * higher-priority tasks.  Skipping a frame preserves the previous sample. */
void AppLcd3d_Timing(AppLcd3dTiming *timing);
/* The view area was painted over (page change): draw it on the next call. */
void AppLcd3d_Invalidate(void);

#endif /* APP_LCD_3D_H */
