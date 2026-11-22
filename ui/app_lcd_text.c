/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd.h"
#include "app_sdram.h"
#include "app_lcd_font.h"
#include "app_lcd_zh.h"
#include <stddef.h>

void AppLcd_DrawChar(uint16_t x, uint16_t y, char c, uint8_t size, uint16_t fg, uint16_t bg)
{
  const uint16_t width_px = AppLcd_PhysicalWidth();
  volatile uint16_t *const fb = (volatile uint16_t *)APP_SDRAM_BASE;
  if (size != 24U) size = 16U;
  const uint8_t width = size / 2U;
  const uint8_t bytes_per_column = (uint8_t)((size + 7U) / 8U);

  if ((c < ' ') || (c > '~')) c = '?';
  if (((uint32_t)x + width > width_px) || ((uint32_t)y + size > AppLcd_PhysicalHeight())) return;
  const uint8_t *const glyph = (size == 24U) ? app_lcd_font24[c - ' '] : app_lcd_font16[c - ' '];

  /* Every pixel of the cell is written, so redrawing a string in place needs
   * no clear and does not flicker.  Row by row: consecutive addresses let
   * the write buffer merge stores to the uncached SDRAM (the font itself is
   * stored column by column). */
  for (uint8_t row = 0U; row < size; row++)
  {
    volatile uint16_t *pixel = &fb[((uint32_t)(y + row) * width_px) + x];
    const uint8_t byte = row / 8U;
    const uint8_t mask = (uint8_t)(0x80U >> (row % 8U));
    for (uint8_t col = 0U; col < width; col++)
      pixel[col] = ((glyph[(col * bytes_per_column) + byte] & mask) != 0U) ? fg : bg;
  }
}

void AppLcd_DrawWide(uint16_t x, uint16_t y, uint16_t code, uint16_t fg, uint16_t bg)
{
  const uint16_t width_px = AppLcd_PhysicalWidth();
  volatile uint16_t *const fb = (volatile uint16_t *)APP_SDRAM_BASE;
  if (((uint32_t)x + 16U > width_px) || ((uint32_t)y + 16U > AppLcd_PhysicalHeight())) return;
  const uint8_t *const glyph = AppLcdZh_Glyph(code);
  if (glyph == NULL)
  {
    AppLcd_DrawChar(x, y, '?', 16U, fg, bg);
    AppLcd_DrawChar((uint16_t)(x + 8U), y, ' ', 16U, fg, bg);
    return;
  }
  uint16_t shade[16];
  AppLcdZh_Palette(fg, bg, shade);
  /* Every pixel of the 16x16 cell is written, like AppLcd_DrawChar. */
  for (uint8_t row = 0U; row < 16U; row++)
  {
    volatile uint16_t *pixel = &fb[((uint32_t)(y + row) * width_px) + x];
    const uint8_t *const bits = &glyph[row * 8U];
    for (uint8_t col = 0U; col < 16U; col += 2U)
    {
      pixel[col] = shade[bits[col / 2U] >> 4];
      pixel[col + 1U] = shade[bits[col / 2U] & 15U];
    }
  }
}

void AppLcd_DrawText(uint16_t x, uint16_t y, const char *text, uint8_t size,
                     uint16_t fg, uint16_t bg)
{
  const uint16_t width_px = AppLcd_PhysicalWidth();
  const uint16_t advance = (size == 24U) ? 12U : 8U;

  while ((*text != '\0') && ((uint32_t)x + advance <= width_px))
  {
    AppLcd_DrawChar(x, y, *text++, size, fg, bg);
    x = (uint16_t)(x + advance);
  }
}
