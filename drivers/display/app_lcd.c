/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_lcd.h"

#include "app_config.h"
#include "app_touch.h"
#include "app_sdram.h"
#include "main.h"

#include <stdio.h>

/* Runtime physical timing; public drawing coordinates are native pixels.
 * 4.3-inch default: GL043056B0-40-N datasheet, 9MHz.
 * 5-inch: Wildfire compatible example, 27MHz. */
static const AppLcdPanel *panel;
static uint32_t first_frame_tick;
_Static_assert(DM_LCD_PANEL==0U || DM_LCD_PANEL==43U || DM_LCD_PANEL==5U, "LCD panel selection");
_Static_assert(DM_LCD_FALLBACK_PANEL==43U || DM_LCD_FALLBACK_PANEL==5U, "LCD fallback");

#define APP_LCD_FRAMEBUFFER  APP_SDRAM_BASE
/* Off-screen drawing area, 1 MB above the 768 KB framebuffer. */
#define APP_LCD_SCRATCH      APP_SDRAM_LCD_SCRATCH
#define APP_LCD_DMA2D_TIMEOUT_MS  50U

_Static_assert(APP_LCD_SCRATCH_PIXELS * sizeof(uint16_t) <= APP_SDRAM_LCD_SCRATCH_SIZE,
               "LCD scratch must fit its MPU region");

/* Panel enable (DISP) and backlight (BL), both active high. */
#define APP_LCD_DISP_PORT  GPIOD
#define APP_LCD_DISP_PIN   GPIO_PIN_4
#define APP_LCD_BL_PORT    GPIOD
#define APP_LCD_BL_PIN     GPIO_PIN_7

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t alternate;
} AppLcdPin;

/* RGB888 bus as wired on the base board (底板原理图 p6).  R3/R6/G3 sit on
 * AF9, everything else on AF14.  PB8/PB9 (B6/B7) are why CAN moved to FDCAN2. */
static const AppLcdPin app_lcd_pins[] = {
  {GPIOH, GPIO_PIN_2, GPIO_AF14_LTDC},  /* R0 */
  {GPIOH, GPIO_PIN_3, GPIO_AF14_LTDC},  /* R1 */
  {GPIOH, GPIO_PIN_8, GPIO_AF14_LTDC},  /* R2 */
  {GPIOB, GPIO_PIN_0, GPIO_AF9_LTDC},   /* R3 */
  {GPIOA, GPIO_PIN_11, GPIO_AF14_LTDC}, /* R4 */
  {GPIOA, GPIO_PIN_12, GPIO_AF14_LTDC}, /* R5 */
  {GPIOB, GPIO_PIN_1, GPIO_AF9_LTDC},   /* R6 */
  {GPIOG, GPIO_PIN_6, GPIO_AF14_LTDC},  /* R7 */
  {GPIOE, GPIO_PIN_5, GPIO_AF14_LTDC},  /* G0 */
  {GPIOE, GPIO_PIN_6, GPIO_AF14_LTDC},  /* G1 */
  {GPIOH, GPIO_PIN_13, GPIO_AF14_LTDC}, /* G2 */
  {GPIOG, GPIO_PIN_10, GPIO_AF9_LTDC},  /* G3 */
  {GPIOH, GPIO_PIN_15, GPIO_AF14_LTDC}, /* G4 */
  {GPIOI, GPIO_PIN_0, GPIO_AF14_LTDC},  /* G5 */
  {GPIOC, GPIO_PIN_7, GPIO_AF14_LTDC},  /* G6 */
  {GPIOI, GPIO_PIN_2, GPIO_AF14_LTDC},  /* G7 */
  {GPIOE, GPIO_PIN_4, GPIO_AF14_LTDC},  /* B0 */
  {GPIOG, GPIO_PIN_12, GPIO_AF14_LTDC}, /* B1 */
  {GPIOD, GPIO_PIN_6, GPIO_AF14_LTDC},  /* B2 */
  {GPIOG, GPIO_PIN_11, GPIO_AF14_LTDC}, /* B3 */
  {GPIOI, GPIO_PIN_4, GPIO_AF14_LTDC},  /* B4 */
  {GPIOA, GPIO_PIN_3, GPIO_AF14_LTDC},  /* B5 */
  {GPIOB, GPIO_PIN_8, GPIO_AF14_LTDC},  /* B6 */
  {GPIOB, GPIO_PIN_9, GPIO_AF14_LTDC},  /* B7 */
  {GPIOG, GPIO_PIN_7, GPIO_AF14_LTDC},  /* CLK */
  {GPIOI, GPIO_PIN_10, GPIO_AF14_LTDC}, /* HSYNC */
  {GPIOI, GPIO_PIN_9, GPIO_AF14_LTDC},  /* VSYNC */
  {GPIOF, GPIO_PIN_10, GPIO_AF14_LTDC}, /* DE */
};

static LTDC_HandleTypeDef app_lcd_ltdc;
static uint8_t app_lcd_ready;

static void AppLcd_PinInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  for (uint32_t i = 0U; i < sizeof(app_lcd_pins) / sizeof(app_lcd_pins[0]); i++)
  {
    gpio.Pin = app_lcd_pins[i].pin;
    gpio.Alternate = app_lcd_pins[i].alternate;
    HAL_GPIO_Init(app_lcd_pins[i].port, &gpio);
  }

  /* Keep the panel and backlight off until the framebuffer holds an image. */
  HAL_GPIO_WritePin(APP_LCD_DISP_PORT, APP_LCD_DISP_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(APP_LCD_BL_PORT, APP_LCD_BL_PIN, GPIO_PIN_RESET);
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0U;
  gpio.Pin = APP_LCD_DISP_PIN | APP_LCD_BL_PIN;
  HAL_GPIO_Init(APP_LCD_DISP_PORT, &gpio);
}

static uint8_t AppLcd_ClockInit(void)
{
  RCC_PeriphCLKInitTypeDef clock = {0};

  clock.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  clock.PLL3.PLL3M = 5U;
  clock.PLL3.PLL3N = panel->pll_n;
  clock.PLL3.PLL3P = 2U;
  clock.PLL3.PLL3Q = 2U;
  clock.PLL3.PLL3R = panel->pll_r;
  clock.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  clock.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  clock.PLL3.PLL3FRACN = 0U;
  return (HAL_RCCEx_PeriphCLKConfig(&clock) == HAL_OK) ? 1U : 0U;
}

static uint8_t AppLcd_LtdcInit(void)
{
  LTDC_LayerCfgTypeDef layer = {0};

  __HAL_RCC_LTDC_CLK_ENABLE();
  __HAL_RCC_DMA2D_CLK_ENABLE();

  app_lcd_ltdc.Instance = LTDC;
  app_lcd_ltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  app_lcd_ltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  app_lcd_ltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  app_lcd_ltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  app_lcd_ltdc.Init.HorizontalSync = panel->hsw - 1U;
  app_lcd_ltdc.Init.VerticalSync = panel->vsw - 1U;
  app_lcd_ltdc.Init.AccumulatedHBP = panel->hsw + panel->hbp - 1U;
  app_lcd_ltdc.Init.AccumulatedVBP = panel->vsw + panel->vbp - 1U;
  app_lcd_ltdc.Init.AccumulatedActiveW = panel->hsw + panel->hbp + panel->width - 1U;
  app_lcd_ltdc.Init.AccumulatedActiveH = panel->vsw + panel->vbp + panel->height - 1U;
  app_lcd_ltdc.Init.TotalWidth = panel->hsw + panel->hbp + panel->width + panel->hfp - 1U;
  app_lcd_ltdc.Init.TotalHeigh = panel->vsw + panel->vbp + panel->height + panel->vfp - 1U;
  app_lcd_ltdc.Init.Backcolor.Blue = 0U;
  app_lcd_ltdc.Init.Backcolor.Green = 0U;
  app_lcd_ltdc.Init.Backcolor.Red = 0U;
  if (HAL_LTDC_Init(&app_lcd_ltdc) != HAL_OK) return 0U;

  layer.WindowX0 = 0U;
  layer.WindowX1 = panel->width;
  layer.WindowY0 = 0U;
  layer.WindowY1 = panel->height;
  layer.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  layer.Alpha = 255U;
  layer.Alpha0 = 0U;
  layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  layer.FBStartAdress = APP_LCD_FRAMEBUFFER;
  layer.ImageWidth = panel->width;
  layer.ImageHeight = panel->height;
  return (HAL_LTDC_ConfigLayer(&app_lcd_ltdc, &layer, 0U) == HAL_OK) ? 1U : 0U;
}

static void AppLcd_Dma2dWait(void)
{
  const uint32_t start = HAL_GetTick();
  while ((DMA2D->CR & DMA2D_CR_START) != 0U)
  {
    if ((HAL_GetTick() - start) > APP_LCD_DMA2D_TIMEOUT_MS)
    {
      DMA2D->CR |= DMA2D_CR_ABORT;
      return;
    }
  }
}

void AppLcd_FillRectIn(uint16_t *buffer, uint16_t pitch, uint16_t x, uint16_t y, uint16_t w,
                       uint16_t h, uint16_t color)
{
  if ((w == 0U) || (h == 0U)) return;
  /* Register-to-memory mode, RGB565 output. */
  DMA2D->CR = DMA2D_CR_MODE_0 | DMA2D_CR_MODE_1;
  DMA2D->OPFCCR = DMA2D_OPFCCR_CM_1;
  DMA2D->OCOLR = color;
  DMA2D->OMAR = (uint32_t)buffer + (((uint32_t)y * pitch) + x) * 2U;
  DMA2D->OOR = (uint32_t)(pitch - w);
  DMA2D->NLR = ((uint32_t)w << DMA2D_NLR_PL_Pos) | h;
  DMA2D->CR |= DMA2D_CR_START;
  AppLcd_Dma2dWait();
}

void AppLcd_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if ((x >= panel->width) || (y >= panel->height) || (w == 0U) || (h == 0U)) return;
  if (w > (panel->width - x)) w = (uint16_t)(panel->width - x);
  if (h > (panel->height - y)) h = (uint16_t)(panel->height - y);
  AppLcd_FillRectIn((uint16_t *)APP_LCD_FRAMEBUFFER,panel->width,x,y,w,h,color);
}

void AppLcd_Blit(const uint16_t *source, uint16_t source_pitch, uint16_t x, uint16_t y,
                 uint16_t w, uint16_t h)
{
  if ((x >= panel->width) || (y >= panel->height) || (w == 0U) || (h == 0U)) return;
  if (w > (panel->width - x)) w = (uint16_t)(panel->width - x);
  if (h > (panel->height - y)) h = (uint16_t)(panel->height - y);
  /* Memory-to-memory, RGB565 in and out: each screen pixel goes straight
   * from the old frame to the new one, so there is no blank in between. */
  DMA2D->CR = 0U;
  DMA2D->FGPFCCR = DMA2D_FGPFCCR_CM_1;
  DMA2D->FGMAR = (uint32_t)source;
  DMA2D->FGOR = (uint32_t)(source_pitch - w);
  DMA2D->OPFCCR = DMA2D_OPFCCR_CM_1;
  DMA2D->OMAR = APP_LCD_FRAMEBUFFER + (((uint32_t)y * panel->width) + x) * 2U;
  DMA2D->OOR = panel->width - w;
  DMA2D->NLR = ((uint32_t)w << DMA2D_NLR_PL_Pos) | h;
  DMA2D->CR |= DMA2D_CR_START;
  AppLcd_Dma2dWait();
}

uint16_t *AppLcd_Scratch(void)
{
  return (uint16_t *)APP_LCD_SCRATCH;
}

uint8_t AppLcd_Init(void (*first_frame)(uint32_t))
{
  app_lcd_ready = 0U;
  if (AppSdram_Init() == 0U)
  {
    printf("LCD: SDRAM check FAILED; display disabled, CAN control continues\r\n");
    return 0U;
  }
  AppLcd_PinInit();
  const uint32_t touch_start=HAL_GetTick();
  (void)AppTouch_Init();
  const uint32_t touch_ms=HAL_GetTick()-touch_start;
  uint16_t rx=0U, ry=0U;
  const uint8_t range_ok=AppTouch_Range(&rx,&ry);
  const uint8_t detected=range_ok ? AppLcdPanel_Detect(rx,ry) : APP_LCD_PANEL_AUTO;
  const uint8_t selected=DM_LCD_PANEL ? DM_LCD_PANEL :
                          (detected ? detected : DM_LCD_FALLBACK_PANEL);
  panel=AppLcdPanel_Get(selected,DM_LCD_43_LEGACY_TIMING);
  printf("LCD: touch GT%s range=%ux%u; panel=%s source=%s probe=%lums\r\n",
         AppTouch_Ready() ? AppTouch_ProductId() : "absent", (unsigned)rx,(unsigned)ry,
         selected==43U ? "4.3in" : "5in",
         DM_LCD_PANEL ? "forced" : (detected ? "touch-config" : "FALLBACK (not detected)"),
         (unsigned long)touch_ms);
  if (!range_ok || detected!=selected)
    printf("LCD: touch range unknown/mismatched; UI touch actions disabled\r\n");
  if ((AppLcd_ClockInit() == 0U) || (AppLcd_LtdcInit() == 0U))
  {
    printf("LCD: LTDC init FAILED; display disabled, CAN control continues\r\n");
    return 0U;
  }

  AppLcd_FillRect(0U, 0U, panel->width, panel->height, 0U);
  /* No RTOS task owns the scratch buffer yet. Present a complete native
   * frame before enabling the panel, including on a network-only trial. */
  first_frame(0U);
  __DSB();
  HAL_GPIO_WritePin(APP_LCD_DISP_PORT, APP_LCD_DISP_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(APP_LCD_BL_PORT, APP_LCD_BL_PIN, GPIO_PIN_SET);
  first_frame_tick = HAL_GetTick();
  app_lcd_ready = 1U;
  printf("LCD: %s %ux%u RGB565 framebuffer=0x%08lX (SDRAM OK), pclk=%uMHz, app_first_frame=%lums\r\n",
         panel->model==43U ? "4.3in" : "5in", (unsigned)panel->width,(unsigned)panel->height,
         (unsigned long)APP_LCD_FRAMEBUFFER,(unsigned)panel->clock_mhz,(unsigned long)first_frame_tick);
  return 1U;
}

uint8_t AppLcd_Ready(void)
{
  return app_lcd_ready;
}

uint32_t AppLcd_FirstFrameTick(void) { return first_frame_tick; }

uint16_t AppLcd_PhysicalWidth(void) { return panel ? panel->width : 800U; }
uint16_t AppLcd_PhysicalHeight(void) { return panel ? panel->height : 480U; }
uint8_t AppLcd_PanelModel(void) { return panel ? panel->model : 5U; }
