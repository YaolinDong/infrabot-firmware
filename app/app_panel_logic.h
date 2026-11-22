/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#ifndef APP_PANEL_LOGIC_H
#define APP_PANEL_LOGIC_H

#include <stdint.h>
#include "app_safety.h"
#include "infrabot/protocol.h"

/* The board status light and keys (app_panel.c) without the hardware, so
 * the host tests can check them. */

/* Light colours (bit 0 red, bit 1 green, bit 2 blue) and blink patterns.
 * Slow ~0.6 Hz and fast 2 Hz, as industrial arm status lights. */
enum
{
  APP_PANEL_OFF = 0U,
  APP_PANEL_RED = 1U,
  APP_PANEL_GREEN = 2U,
  APP_PANEL_YELLOW = 3U,
  APP_PANEL_BLUE = 4U,
  APP_PANEL_WHITE = 7U
};
enum { APP_PANEL_SOLID, APP_PANEL_SLOW, APP_PANEL_FAST };
#define APP_PANEL_SLOW_PERIOD_MS 1600U
#define APP_PANEL_FAST_PERIOD_MS 500U

typedef struct
{
  uint8_t color;
  uint8_t blink;
} AppPanelLight;

typedef struct
{
  uint8_t arm_power_lost; /* J1-J7 silent: no arm power (or not yet at boot) */
  uint8_t starting;       /* bring-up: the control task is not running yet */
  uint8_t fault;          /* CAN bus fault, or an enabled drive reports a fault */
  uint8_t stop_reason;    /* latched INFRABOT_STOP_* */
  uint8_t setup;          /* staged enable, profile/self-check, register access */
  uint8_t enabled;        /* any drive enabled */
  uint8_t control_mode;   /* INFRABOT_CONTROL_* */
} AppPanelInputs;

/* One light shows the most severe state:
 *   red slow      arm without power, re-probing
 *   white slow    bring-up
 *   red           fault, or a stop RESUME cannot clear (needs a re-enable)
 *   yellow slow   stopped and holding, RESUME (KEY2) continues
 *   blue fast     enabling / applying the profile / register access
 *   white         all drives disabled
 *   blue slow     gravity FREE (hand guiding)
 *   green         a PC's RT/NRT motion mode
 *   blue          enabled and holding (IDLE, gravity LOCK, joint/Cartesian impedance) */
static inline AppPanelLight AppPanel_Light(const AppPanelInputs *in)
{
  AppPanelLight light = {APP_PANEL_BLUE, APP_PANEL_SOLID};
  if (in->arm_power_lost) { light.color = APP_PANEL_RED; light.blink = APP_PANEL_SLOW; }
  else if (in->starting) { light.color = APP_PANEL_WHITE; light.blink = APP_PANEL_SLOW; }
  else if (in->fault) light.color = APP_PANEL_RED;
  else if (in->enabled && in->stop_reason != INFRABOT_STOP_NONE)
  {
    if (AppSafety_Resumable(in->stop_reason))
    {
      light.color = APP_PANEL_YELLOW;
      light.blink = APP_PANEL_SLOW;
    }
    else light.color = APP_PANEL_RED;
  }
  else if (in->setup) light.blink = APP_PANEL_FAST;
  else if (!in->enabled) light.color = APP_PANEL_WHITE;
  else if (in->control_mode == INFRABOT_CONTROL_GRAVITY_FREE) light.blink = APP_PANEL_SLOW;
  else if (in->control_mode == INFRABOT_CONTROL_RT_JOINT ||
           in->control_mode == INFRABOT_CONTROL_RT_CARTESIAN ||
           in->control_mode == INFRABOT_CONTROL_NRT_JOINT ||
           in->control_mode == INFRABOT_CONTROL_NRT_CARTESIAN)
    light.color = APP_PANEL_GREEN;
  return light;
}

/* The colour lit at now_ms: blinking lights are on for the first half of
 * their period. */
static inline uint8_t AppPanel_LightColor(AppPanelLight light, uint32_t now_ms)
{
  if (light.blink == APP_PANEL_SOLID) return light.color;
  const uint32_t period = (light.blink == APP_PANEL_SLOW) ? APP_PANEL_SLOW_PERIOD_MS :
                          APP_PANEL_FAST_PERIOD_MS;
  return ((now_ms % period) < period / 2U) ? light.color : (uint8_t)APP_PANEL_OFF;
}

/* A key sampled every period_ms; it changes state after
 * APP_PANEL_DEBOUNCE_SAMPLES equal samples. */
#define APP_PANEL_DEBOUNCE_SAMPLES 3U
enum { APP_PANEL_KEY_NONE, APP_PANEL_KEY_PRESS, APP_PANEL_KEY_LONG };

typedef struct
{
  uint8_t down;     /* debounced state */
  uint8_t count;    /* samples that disagree with it */
  uint8_t long_sent;
  uint16_t held_ms;
} AppPanelKey;

/* A key already down at boot reports nothing until released and pressed
 * again. */
static inline void AppPanel_KeyInit(AppPanelKey *key, uint8_t down)
{
  key->down = down != 0U;
  key->count = 0U;
  key->long_sent = key->down;
  key->held_ms = 0U;
}

/* PRESS on the debounced press, LONG once when held long_ms (0: never). */
static inline uint8_t AppPanel_KeySample(AppPanelKey *key, uint8_t down, uint16_t period_ms,
                                         uint16_t long_ms)
{
  if ((down != 0U) == key->down) key->count = 0U;
  else if (++key->count >= APP_PANEL_DEBOUNCE_SAMPLES)
  {
    key->down = down != 0U;
    key->count = 0U;
    key->long_sent = 0U;
    key->held_ms = 0U;
    return key->down ? APP_PANEL_KEY_PRESS : APP_PANEL_KEY_NONE;
  }
  if (key->down && long_ms != 0U && !key->long_sent)
  {
    key->held_ms = (uint16_t)(key->held_ms + period_ms);
    if (key->held_ms >= long_ms)
    {
      key->long_sent = 1U;
      return APP_PANEL_KEY_LONG;
    }
  }
  return APP_PANEL_KEY_NONE;
}

#endif /* APP_PANEL_LOGIC_H */
