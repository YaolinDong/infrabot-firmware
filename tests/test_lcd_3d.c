/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Exercise the actual private rasterizer and complete frame lifecycle. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_lcd_3d.c"

MockDwt lcd_dwt;
MockMotor dm_motors[DM_MOTOR_COUNT];

static _Alignas(32) uint16_t cpu_image[APP_LCD_SCRATCH_PIXELS];
static uint16_t dma_image[APP_LCD_SCRATCH_PIXELS];
static uint16_t display_image[APP_LCD_SCRATCH_PIXELS];
static uint8_t coverage[APP_LCD_SCRATCH_PIXELS];
#if APP_LCD_FAST_BUFFER
static uint16_t fast_dma_image[C3D_FAST_PIXELS];
#endif
static uint32_t cache_offset, cache_size;
static uint16_t fill_pitch, fill_x, fill_y, fill_w, fill_h;
static unsigned phase, blits;
static unsigned sram_blits, sdram_blits;

uint16_t *AppLcd_Scratch(void) { return cpu_image; }

void SCB_InvalidateDCache_by_Addr(volatile void *address, int32_t size)
{
  assert(APP_LCD_SCRATCH_CACHE);
  assert(((uintptr_t)address & 31U) == 0U && size > 0 && (size & 31) == 0);
  const uintptr_t offset = (uintptr_t)address - (uintptr_t)cpu_image;
  assert(offset + (uint32_t)size <= sizeof(cpu_image));
  assert(phase == 0 || phase == 2);
  if (phase == 0) { cache_offset = (uint32_t)offset; cache_size = (uint32_t)size; }
  else assert(offset == cache_offset && (uint32_t)size == cache_size);
  memcpy((uint8_t *)cpu_image + offset, (uint8_t *)dma_image + offset, (size_t)size);
  ++phase;
  lcd_dwt.CYCCNT += 7;
}

void SCB_CleanDCache_by_Addr(volatile void *address, int32_t size)
{
#if APP_LCD_FAST_BUFFER
  if (address == (void *)fast_pixels)
  {
    assert(phase == 0 && size > 0 && (size & 31) == 0);
    assert((uint32_t)size <= sizeof(fast_pixels));
    memcpy(fast_dma_image, fast_pixels, (size_t)size);
    cache_offset = 0;
    cache_size = (uint32_t)size;
    phase = 4;
    lcd_dwt.CYCCNT += 13;
    return;
  }
#endif
  assert(APP_LCD_SCRATCH_CACHE && phase == 3);
  assert((uintptr_t)address - (uintptr_t)cpu_image == cache_offset && (uint32_t)size == cache_size);
  memcpy((uint8_t *)dma_image + cache_offset, (uint8_t *)cpu_image + cache_offset, (size_t)size);
  phase = 4;
  lcd_dwt.CYCCNT += 13;
}

void AppLcd_FillRectIn(uint16_t *buffer, uint16_t pitch, uint16_t x, uint16_t y,
                       uint16_t w, uint16_t h, uint16_t color)
{
  assert(buffer == cpu_image && x + w <= pitch && w > 0 && h > 0);
  assert(phase == (APP_LCD_SCRATCH_CACHE ? 1U : 0U));
  fill_pitch = pitch; fill_x = x; fill_y = y; fill_w = w; fill_h = h;
  if (APP_LCD_SCRATCH_CACHE)
  {
    const uint32_t first = ((uint32_t)y * pitch + x) * 2U;
    const uint32_t last = (((uint32_t)y + h - 1U) * pitch + x + w) * 2U;
    assert(cache_offset == (first & ~31U));
    assert(cache_offset + cache_size == ((last + 31U) & ~31U));
  }
  uint16_t *memory = APP_LCD_SCRATCH_CACHE ? dma_image : cpu_image;
  for (uint32_t row = y; row < (uint32_t)y + h; ++row)
    for (uint32_t col = x; col < (uint32_t)x + w; ++col) memory[row * pitch + col] = color;
  phase = 2;
  lcd_dwt.CYCCNT += 11;
}

void AppLcd_Blit(const uint16_t *source, uint16_t pitch, uint16_t x, uint16_t y,
                 uint16_t w, uint16_t h)
{
  assert(x + w <= APP_LCD_WIDTH && y + h <= APP_LCD_HEIGHT);
  const uint16_t *dma_source;
#if APP_LCD_FAST_BUFFER
  if (source == fast_pixels)
  {
    assert(pitch == w && (uint32_t)w * h <= C3D_FAST_PIXELS);
    assert(phase == 4 && cache_offset == 0);
    assert(cache_size == (((uint32_t)w * h * 2U + 31U) & ~31U));
    dma_source = fast_dma_image;
    ++sram_blits;
  }
  else
#endif
  {
    assert(source == cpu_image + (uint32_t)fill_y * fill_pitch + fill_x);
    assert(pitch == fill_pitch && w == fill_w && h == fill_h);
    assert(phase == (APP_LCD_SCRATCH_CACHE ? 4U : 2U));
    dma_source = APP_LCD_SCRATCH_CACHE ? dma_image + (source - cpu_image) : source;
    ++sdram_blits;
  }
  for (unsigned row = 0; row < h; ++row)
  {
    for (unsigned col = 0; col < w; ++col)
      assert(++coverage[(y + row) * APP_LCD_WIDTH + x + col] == 1);
    assert(memcmp(source + row * pitch, dma_source + row * pitch, (size_t)w * 2U) == 0);
    memcpy(display_image + (y + row) * APP_LCD_WIDTH + x, dma_source + row * pitch, (size_t)w * 2U);
  }
  phase = 0;
  ++blits;
  lcd_dwt.CYCCNT += 17;
}

static uint8_t RenderChecked(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  memset(coverage, 0, sizeof(coverage));
  const unsigned before = blits;
  const uint8_t rendered = AppLcd3d_Render(x, y, w, h);
  unsigned pixels = 0;
  for (unsigned n = 0; n < APP_LCD_SCRATCH_PIXELS; ++n) pixels += coverage[n];
  if (rendered)
  {
    assert(pixels == frame_timing.updated_pixels);
    assert(blits - before == frame_timing.tiles);
    assert(frame_timing.sram_buffer == APP_LCD_FAST_BUFFER);
  }
  else assert(pixels == 0 && blits == before);
  return rendered;
}

static void TestSpans(void)
{
  /* Odd pitches and both halfword alignments, including empty spans. */
  _Alignas(4) uint16_t memory[96];
  for (int pitch = 15; pitch <= 16; ++pitch)
    for (int first = -3; first <= pitch + 1; ++first)
      for (int last = first; last <= pitch + 2; ++last)
      {
        for (unsigned i = 0; i < 96; ++i) memory[i] = 0x5AA5;
        const Canvas c = {memory + 1, (int16_t)pitch, 3, {0,0,(int16_t)pitch,3}, 0, 0};
        FillSpan(&c, 1, (float)first, (float)last, (float)first, (float)last, 0x1234);
        for (int i = 0; i < 96; ++i)
        {
          const int x = i - 1 - pitch;
          const uint16_t expected = (x >= 0 && x < pitch && x >= first && x <= last)
                                    ? 0x1234 : 0x5AA5;
          assert(memory[i] == expected);
        }
      }
}

static void TestTriangles(void)
{
  uint16_t memory[18 * 18];
  const Canvas c = {memory, 18, 18, {0,0,18,18}, 0, 0};
  const Projected p[3] = {{1, 1, 0}, {7, 1, 0}, {1, 7, 0}};
  const unsigned permutations[6][3] = {{0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0}};
  for (unsigned n = 0; n < 6; ++n)
  {
    memset(memory, 0, sizeof(memory));
    FillTriangle(&c, p[permutations[n][0]], p[permutations[n][1]], p[permutations[n][2]], 1);
    for (int y = 0; y < 18; ++y)
      for (int x = 0; x < 18; ++x)
        assert(memory[y * 18 + x] == (uint16_t)(x >= 1 && y >= 1 && x + y <= 8));
  }

  memset(memory, 0, sizeof(memory));
  FillTriangle(&c, (Projected){-4,-4,0}, (Projected){8,-4,0}, (Projected){-4,8,0}, 1);
  for (int y = 0; y < 18; ++y)
    for (int x = 0; x < 18; ++x)
      assert(memory[y * 18 + x] == (uint16_t)(x + y <= 4));

  /* Shared diagonal and fractional first/last rows must leave no holes. */
  memset(memory, 0, sizeof(memory));
  FillTriangle(&c, (Projected){2,2.2f,0}, (Projected){12,2.2f,0}, (Projected){12,12.8f,0}, 1);
  FillTriangle(&c, (Projected){2,2.2f,0}, (Projected){12,12.8f,0}, (Projected){2,12.8f,0}, 1);
  for (int y = 0; y < 18; ++y)
    for (int x = 0; x < 18; ++x)
      assert(memory[y * 18 + x] == (uint16_t)(x >= 2 && x <= 12 && y >= 2 && y <= 13));

  /* A subpixel-height face must not extrapolate its nearly flat edge. */
  memset(memory, 0, sizeof(memory));
  FillTriangle(&c, (Projected){2,2.1f,0}, (Projected){12,2.2f,0}, (Projected){7,2.3f,0}, 1);
  for (int i = 0; i < 18 * 18; ++i) assert(memory[i] == (uint16_t)(i == 2 * 18 + 2));

  memset(memory, 0, sizeof(memory));
  const Canvas clipped = {memory, 18, 18, {3,4,11,13}, 0, 0};
  FillTriangle(&clipped, (Projected){-20,-20,0}, (Projected){60,-20,0}, (Projected){-20,60,0}, 1);
  for (int y = 0; y < 18; ++y)
    for (int x = 0; x < 18; ++x)
      assert(memory[y * 18 + x] == (uint16_t)(x >= 3 && x < 11 && y >= 4 && y < 13));
}

static void TestTriangleTileSeams(void)
{
  enum { W = 61, H = 137, ROWS = 7 };
  uint16_t full[W * H], tile[W * ROWS];
  Projected triangles[60][3];
  uint32_t rng = 1701U;
  for (unsigned n = 0; n < 60; ++n)
    for (unsigned v = 0; v < 3; ++v)
    {
      rng = rng * 1664525U + 1013904223U;
      triangles[n][v].sx = (float)(rng % 10000U) / 100.0f - 20.0f;
      rng = rng * 1664525U + 1013904223U;
      triangles[n][v].sy = (float)(rng % 18000U) / 100.0f - 20.0f;
      triangles[n][v].depth = 0;
    }
  memset(full, 0, sizeof(full));
  const Canvas c = {full, W, H, {0,0,W,H}, 0, 0};
  for (unsigned n = 0; n < 60; ++n)
    FillTriangle(&c, triangles[n][0], triangles[n][1], triangles[n][2], (uint16_t)(n + 1));
  for (int y = 0; y < H; y += ROWS)
  {
    const int end = y + ROWS < H ? y + ROWS : H;
    const Canvas sub = {tile, W, (int16_t)(end - y), {0,(int16_t)y,W,(int16_t)end}, 0,(int16_t)y};
    memset(tile, 0, sizeof(tile));
    for (unsigned n = 0; n < 60; ++n)
      FillTriangle(&sub, triangles[n][0], triangles[n][1], triangles[n][2], (uint16_t)(n + 1));
    assert(memcmp(tile, full + y * W, (size_t)(end - y) * W * 2U) == 0);
  }
}

/* Independent full-size CPU surface: compares compact storage offsets,
 * clear coverage and DMA submission against an ordinary full canvas. */
static void CheckReference(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  static uint16_t reference[APP_LCD_SCRATCH_PIXELS];
  const Canvas c = {reference, (int16_t)w, (int16_t)h, {0,0,(int16_t)w,(int16_t)h}, 0, 0};
  for (uint32_t n = 0; n < (uint32_t)w * h; ++n) reference[n] = C3D_BG;
  AppLcd3d_DrawGrid(&c);
  for (uint32_t n = 0; n < solid_face_count; ++n)
  {
    const Face *f = &solid_faces[n];
    FillTriangle(&c, solid_vertices[f->a], solid_vertices[f->b], solid_vertices[f->c], f->color);
  }
  AppLcd3d_DrawToolAxes(&c);
  for (uint32_t row = 0; row < h; ++row)
    assert(memcmp(reference + row * w, display_image + (y + row) * APP_LCD_WIDTH + x,
                   (size_t)w * 2U) == 0);
}

static void TestFrames(void)
{
  /* Full-frame tiles, odd compact sizes and either side of 128 KiB. */
  const uint16_t sizes[][2] = {{584,431}, {31,33}, {800,480}, {256,256}, {257,256}, {255,257}};
  const unsigned count = sizeof(sizes) / sizeof(sizes[0]);
  for (unsigned n = 0; n < count; ++n)
  {
    const uint16_t w = sizes[n][0], h = sizes[n][1];
    memset(cpu_image, 0xA5, sizeof(cpu_image));
    memset(dma_image, 0x5A, sizeof(dma_image));
    AppLcd3d_Invalidate();
    for (unsigned frame = 0; frame < 4; ++frame)
    {
      AppLcd3d_Invalidate();
      AppLcd3d_SetView((AppLcd3dView)frame);
      assert(RenderChecked(0, 0, w, h) == 1);
      CheckReference(0, 0, w, h);
      assert(phase == 0);
      AppLcd3dTiming t, skipped;
      AppLcd3d_Timing(&t);
      assert(t.draw_cycles == t.clear_cycles + t.raster_cycles + t.clean_cycles + t.blit_cycles);
      assert(t.clear_cycles == t.tiles * (t.sram_buffer ? 0U : (APP_LCD_SCRATCH_CACHE ? 25U : 11U)));
      assert(t.clean_cycles == t.tiles * ((t.sram_buffer || APP_LCD_SCRATCH_CACHE) ? 13U : 0U));
      assert(t.blit_cycles == t.tiles * 17U);
      if (APP_LCD_FAST_BUFFER && (uint32_t)w * h > C3D_FAST_PIXELS) assert(t.tiles > 1);
      assert(t.updated_pixels == (uint32_t)w * h);
      const uint32_t clock_before = lcd_dwt.CYCCNT;
      assert(RenderChecked(0, 0, w, h) == 0);
      assert(lcd_dwt.CYCCNT == clock_before);
      AppLcd3d_Timing(&skipped);
      assert(memcmp(&t, &skipped, sizeof(t)) == 0);
    }
  }
  assert(blits >= count * 4U);
  assert(RenderChecked(0, 0, 0, 480) == 0);
  assert(RenderChecked(0, 0, 800, 481) == 0);
}

static void TestPartialFrames(void)
{
  static uint16_t partial[APP_LCD_SCRATCH_PIXELS];
  unsigned partial_frames = 0;
  for (unsigned view = 0; view < 4; ++view)
  {
    AppLcd3d_SetView((AppLcd3dView)view);
    AppLcd3d_Invalidate();
    for (unsigned frame = 0; frame < 20; ++frame)
    {
      /* Include rapid reversals and near-limit poses to exercise the union
       * of old/new bounds, clipping, odd widths and cache-line neighbours. */
      const float phase_rad = (float)frame * 0.47f;
      const float q[7] = {1.5f * sinf(phase_rad), 1.4f + 1.3f * sinf(phase_rad),
                          sinf(phase_rad + 0.5f), 1.4f + 1.3f * cosf(phase_rad),
                          sinf(phase_rad), cosf(phase_rad), sinf(phase_rad)};
      for (unsigned i = 0; i < 7; ++i) dm_motors[i].position = q[i] * DM_MOTOR_DIRECTION(i);
      assert(RenderChecked(13, 49, 584, 431) == 1);
      CheckReference(13, 49, 584, 431);
      AppLcd3dTiming timing;
      AppLcd3d_Timing(&timing);
      if (frame == 0 || !APP_LCD_PARTIAL_REFRESH) assert(timing.updated_pixels == 584U * 431U);
      else if (timing.updated_pixels < 584U * 431U) ++partial_frames;
      memcpy(partial, display_image, sizeof(partial));
      /* Force an independent full repaint of the same scene.  The display
       * must match exactly, including pixels outside the updated rectangle. */
      AppLcd3d_Invalidate();
      assert(RenderChecked(13, 49, 584, 431) == 1);
      assert(memcmp(partial, display_image, sizeof(partial)) == 0);
    }
  }
  if (APP_LCD_PARTIAL_REFRESH) assert(partial_frames > 60);
  /* A changed viewport position or size also invalidates the static grid. */
  assert(RenderChecked(14, 49, 584, 431) == 1);
  assert(RenderChecked(14, 49, 583, 431) == 1);
  assert(RenderChecked(14, 49, 583, 431) == 0);
  printf("80 full/partial frame comparisons passed (%u reduced updates)\n", partial_frames);
}

static void TestBufferTransitions(void)
{
  for (unsigned i = 0; i < 7; ++i) dm_motors[i].position = 0;
  AppLcd3d_SetView(APP_LCD3D_VIEW_HOME);
  const unsigned before = sram_blits;
  for (unsigned n = 0; n < 12; ++n)
  {
    /* Camera changes use several tiles; the next small pose update needs
     * one tile.  Reusing the buffer must never leak pixels from other tiles. */
    AppLcd3d_Orbit(1, 0);
    assert(RenderChecked(0, 49, 584, 431));
    if (APP_LCD_FAST_BUFFER) assert(frame_timing.tiles > 1);
    CheckReference(0, 49, 584, 431);
    dm_motors[0].position += 0.01f;
    assert(RenderChecked(0, 49, 584, 431));
    if (APP_LCD_FAST_BUFFER && APP_LCD_PARTIAL_REFRESH) assert(frame_timing.sram_buffer);
    CheckReference(0, 49, 584, 431);
  }
  if (APP_LCD_FAST_BUFFER) assert(sram_blits > before + 24U && sdram_blits == 0);
  else assert(sdram_blits > 0);
}

static void TestOrbitFrames(void)
{
  AppLcd3d_SetView(APP_LCD3D_VIEW_HOME);
  AppLcd3d_Invalidate();
  unsigned reduced = 0;
  for (unsigned frame = 0; frame < 160; ++frame)
  {
    /* Rotate through both pitch clamps and >360 degrees of yaw while the
     * arm changes pose; check grid erasure and every model tile seam. */
    AppLcd3d_Orbit(11, frame < 80 ? 5 : -5);
    for (unsigned i = 0; i < 7; ++i)
      dm_motors[i].position = 1.3f * sinf((float)frame * 0.31f + (float)i) * DM_MOTOR_DIRECTION(i);
    assert(RenderChecked(13, 49, 583, 431));
    CheckReference(13, 49, 583, 431);
    if (frame_timing.updated_pixels < 583U * 431U) ++reduced;
  }
  if (APP_LCD_PARTIAL_REFRESH) assert(reduced > 50);
  printf("160 moving/orbiting frames match full canvas (%u reduced scene updates)\n", reduced);
}

int main(void)
{
  TestSpans();
  TestTriangles();
  TestTriangleTileSeams();
  TestFrames();
  TestPartialFrames();
  TestBufferTransitions();
  TestOrbitFrames();
  puts("LCD raster coverage, clipping, packed stores and cache/DMA lifecycle passed");
  return 0;
}
