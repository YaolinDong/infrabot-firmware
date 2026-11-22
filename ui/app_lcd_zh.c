/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd_zh.h"

#include <stddef.h>

#include "app_lcd_zh_font.inc"

uint16_t AppLcdZh_Next(const char **text)
{
  const unsigned char *s = (const unsigned char *)*text;
  if (s[0] == 0U) return 0U;
  if (s[0] < 0x80U) { *text += 1; return s[0]; }
  if (s[0] >= 0xC2U && s[0] <= 0xDFU && (s[1] & 0xC0U) == 0x80U)
  {
    *text += 2;
    return (uint16_t)(((s[0] & 0x1FU) << 6) | (s[1] & 0x3FU));
  }
  if (s[0] >= 0xE0U && s[0] <= 0xEFU && (s[1] & 0xC0U) == 0x80U && (s[2] & 0xC0U) == 0x80U)
  {
    const uint16_t code = (uint16_t)(((s[0] & 0x0FU) << 12) | ((s[1] & 0x3FU) << 6) | (s[2] & 0x3FU));
    if (code >= 0x800U && (code < 0xD800U || code > 0xDFFFU)) { *text += 3; return code; }
  }
  *text += 1;
  return '?';
}

const uint8_t *AppLcdZh_Glyph(uint16_t code)
{
  size_t lo = 0U, hi = APP_LCD_ZH_GLYPHS;
  while (lo < hi)
  {
    const size_t mid = (lo + hi) / 2U;
    if (app_lcd_zh_codes[mid] == code) return app_lcd_zh_glyphs[mid];
    if (app_lcd_zh_codes[mid] < code) lo = mid + 1U;
    else hi = mid;
  }
  return NULL;
}

unsigned AppLcdZh_Columns(const char *text)
{
  unsigned columns = 0U;
  for (uint16_t c; (c = AppLcdZh_Next(&text)) != 0U;) columns += AppLcdZh_Width(c);
  return columns;
}

unsigned AppLcdZh_Encode(uint16_t code, char *out)
{
  if (code < 0x80U) { out[0] = (char)code; return 1U; }
  if (code < 0x800U)
  {
    out[0] = (char)(0xC0U | (code >> 6));
    out[1] = (char)(0x80U | (code & 0x3FU));
    return 2U;
  }
  out[0] = (char)(0xE0U | (code >> 12));
  out[1] = (char)(0x80U | ((code >> 6) & 0x3FU));
  out[2] = (char)(0x80U | (code & 0x3FU));
  return 3U;
}

void AppLcdZh_Palette(uint16_t fg, uint16_t bg, uint16_t out[16])
{
  const int fr = fg >> 11, fgreen = (fg >> 5) & 63, fb = fg & 31;
  const int br = bg >> 11, bgreen = (bg >> 5) & 63, bb = bg & 31;
  for (int k = 0; k < 16; ++k)
  {
    const int r = br + ((fr - br) * k + (fr >= br ? 7 : -7)) / 15;
    const int g = bgreen + ((fgreen - bgreen) * k + (fgreen >= bgreen ? 7 : -7)) / 15;
    const int b = bb + ((fb - bb) * k + (fb >= bb ? 7 : -7)) / 15;
    out[k] = (uint16_t)((r << 11) | (g << 5) | b);
  }
}
