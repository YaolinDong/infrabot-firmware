/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_panel_logic.h"

#include <assert.h>
#include <stdio.h>

static AppPanelLight light(AppPanelInputs in)
{
  return AppPanel_Light(&in);
}

static void expect(AppPanelInputs in, uint8_t color, uint8_t blink)
{
  const AppPanelLight l = light(in);
  assert(l.color == color && l.blink == blink);
}

static void test_light(void)
{
  const AppPanelInputs idle = {0, 0, 0, INFRABOT_STOP_NONE, 0, 0, INFRABOT_CONTROL_IDLE};
  AppPanelInputs in = idle;
  expect(in, APP_PANEL_WHITE, APP_PANEL_SOLID);

  in.enabled = 1U;
  expect(in, APP_PANEL_BLUE, APP_PANEL_SOLID);
  in.control_mode = INFRABOT_CONTROL_GRAVITY_LOCK;
  expect(in, APP_PANEL_BLUE, APP_PANEL_SOLID);
  in.control_mode = INFRABOT_CONTROL_GRAVITY_FREE;
  expect(in, APP_PANEL_BLUE, APP_PANEL_SLOW);
  const uint8_t motion[] = {INFRABOT_CONTROL_RT_JOINT, INFRABOT_CONTROL_RT_CARTESIAN,
                            INFRABOT_CONTROL_NRT_JOINT, INFRABOT_CONTROL_NRT_CARTESIAN};
  for (unsigned i = 0U; i < sizeof(motion); ++i)
  {
    in.control_mode = motion[i];
    expect(in, APP_PANEL_GREEN, APP_PANEL_SOLID);
  }

  /* Setup blinks blue whether or not a drive is already enabled. */
  in.setup = 1U;
  expect(in, APP_PANEL_BLUE, APP_PANEL_FAST);
  in.enabled = 0U;
  expect(in, APP_PANEL_BLUE, APP_PANEL_FAST);

  /* A latched stop: yellow when RESUME can clear it, else red. */
  in = idle;
  in.enabled = 1U;
  in.setup = 1U;
  const uint8_t resumable[] = {INFRABOT_STOP_REQUESTED, INFRABOT_STOP_STREAM_LOST,
                               INFRABOT_STOP_MOVE_LOST};
  for (unsigned i = 0U; i < sizeof(resumable); ++i)
  {
    in.stop_reason = resumable[i];
    expect(in, APP_PANEL_YELLOW, APP_PANEL_SLOW);
  }
  const uint8_t faults[] = {INFRABOT_STOP_MOVE_FAILED, INFRABOT_STOP_NOT_FOLLOWING,
                            INFRABOT_STOP_INVALID_TARGET, INFRABOT_STOP_FEEDBACK_LOST,
                            INFRABOT_STOP_DRIVE_FAULT, INFRABOT_STOP_TORQUE_LIMIT};
  for (unsigned i = 0U; i < sizeof(faults); ++i)
  {
    in.stop_reason = faults[i];
    expect(in, APP_PANEL_RED, APP_PANEL_SOLID);
  }
  /* The reason stays latched after a DISABLE: no drive holds, so no stop. */
  in.enabled = 0U;
  in.setup = 0U;
  expect(in, APP_PANEL_WHITE, APP_PANEL_SOLID);

  /* Severity order: arm power > bring-up > fault > stop. */
  in = idle;
  in.enabled = 1U;
  in.stop_reason = INFRABOT_STOP_REQUESTED;
  in.fault = 1U;
  expect(in, APP_PANEL_RED, APP_PANEL_SOLID);
  in.starting = 1U;
  expect(in, APP_PANEL_WHITE, APP_PANEL_SLOW);
  in.arm_power_lost = 1U;
  expect(in, APP_PANEL_RED, APP_PANEL_SLOW);

  /* Blinking: on for the first half of each period. */
  const AppPanelLight slow = {APP_PANEL_YELLOW, APP_PANEL_SLOW};
  const AppPanelLight fast = {APP_PANEL_BLUE, APP_PANEL_FAST};
  const AppPanelLight solid = {APP_PANEL_GREEN, APP_PANEL_SOLID};
  assert(AppPanel_LightColor(slow, 0U) == APP_PANEL_YELLOW);
  assert(AppPanel_LightColor(slow, 799U) == APP_PANEL_YELLOW);
  assert(AppPanel_LightColor(slow, 800U) == APP_PANEL_OFF);
  assert(AppPanel_LightColor(slow, 1600U) == APP_PANEL_YELLOW);
  assert(AppPanel_LightColor(fast, 249U) == APP_PANEL_BLUE);
  assert(AppPanel_LightColor(fast, 250U) == APP_PANEL_OFF);
  assert(AppPanel_LightColor(solid, 800U) == APP_PANEL_GREEN);
  puts("panel light: severity order, stop colours match RESUME, blink phases: OK");
}

/* Samples until the key reports something; returns the event and counts the samples. */
static uint8_t run(AppPanelKey *key, uint8_t down, unsigned samples, uint16_t long_ms,
                   unsigned *at)
{
  for (unsigned i = 1U; i <= samples; ++i)
  {
    const uint8_t event = AppPanel_KeySample(key, down, 10U, long_ms);
    if (event != APP_PANEL_KEY_NONE)
    {
      *at = i;
      return event;
    }
  }
  *at = 0U;
  return APP_PANEL_KEY_NONE;
}

static void test_keys(void)
{
  AppPanelKey key;
  unsigned at;

  /* PRESS after the debounce, once; release reports nothing. */
  AppPanel_KeyInit(&key, 0U);
  assert(run(&key, 1U, 10U, 0U, &at) == APP_PANEL_KEY_PRESS && at == APP_PANEL_DEBOUNCE_SAMPLES);
  assert(run(&key, 1U, 500U, 0U, &at) == APP_PANEL_KEY_NONE);
  assert(run(&key, 0U, 10U, 0U, &at) == APP_PANEL_KEY_NONE);

  /* Bounces shorter than the debounce are ignored. */
  AppPanel_KeyInit(&key, 0U);
  for (unsigned i = 0U; i < 20U; ++i)
  {
    assert(AppPanel_KeySample(&key, 1U, 10U, 0U) == APP_PANEL_KEY_NONE);
    assert(AppPanel_KeySample(&key, 0U, 10U, 0U) == APP_PANEL_KEY_NONE);
  }

  /* LONG once after the hold time, counted from the debounced press. */
  AppPanel_KeyInit(&key, 0U);
  assert(run(&key, 1U, 10U, 1000U, &at) == APP_PANEL_KEY_PRESS);
  assert(run(&key, 1U, 200U, 1000U, &at) == APP_PANEL_KEY_LONG && at == 100U);
  assert(run(&key, 1U, 500U, 1000U, &at) == APP_PANEL_KEY_NONE);
  /* Released early: no LONG; the next press starts a new count. */
  assert(run(&key, 0U, 10U, 1000U, &at) == APP_PANEL_KEY_NONE);
  assert(run(&key, 1U, 10U, 1000U, &at) == APP_PANEL_KEY_PRESS);
  assert(run(&key, 1U, 50U, 1000U, &at) == APP_PANEL_KEY_NONE);
  assert(run(&key, 0U, 10U, 1000U, &at) == APP_PANEL_KEY_NONE);
  assert(run(&key, 1U, 10U, 1000U, &at) == APP_PANEL_KEY_PRESS);
  assert(run(&key, 1U, 200U, 1000U, &at) == APP_PANEL_KEY_LONG && at == 100U);

  /* Down at boot (stuck or held): nothing until released and pressed again. */
  AppPanel_KeyInit(&key, 1U);
  assert(run(&key, 1U, 500U, 1000U, &at) == APP_PANEL_KEY_NONE);
  assert(run(&key, 0U, 10U, 1000U, &at) == APP_PANEL_KEY_NONE);
  assert(run(&key, 1U, 10U, 1000U, &at) == APP_PANEL_KEY_PRESS);
  puts("panel keys: debounce, single PRESS, one LONG per hold, held-at-boot ignored: OK");
}

int main(void)
{
  test_light();
  test_keys();
  return 0;
}
