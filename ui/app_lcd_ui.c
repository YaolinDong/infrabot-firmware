/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "app_lcd_ui.h"

#include "app_config.h"
#include "app_ethernet.h"
#include "app_lcd.h"
#include "app_build_info.h"
#include "app_lcd_boot.h"
#include "app_lcd_3d.h"
#include "app_lcd_mesh.h"
#include "app_lcd_schedule.h"
#include "app_persist.h"
#include "app_safety.h"
#include "app_gravity.h"
#include "app_impedance.h"
#include "app_admittance.h"
#include "app_local_control.h"
#include "app_lcd_slider.h"
#include "app_lcd_zh.h"
#include "app_startup.h"
#include "arm_gravity.h"
#include "app_state.h"
#include "app_tasks.h"
#include "app_touch.h"
#include "infrabot_ik_mcu.h"
#include "main.h"
#include "robot_controller.h"
#include "task.h"

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#define UI_BG           APP_LCD_RGB565(0U, 0U, 0U)
#define UI_BLACK        APP_LCD_RGB565(0U, 0U, 0U)
#define UI_TEXT         APP_LCD_RGB565(230U, 230U, 230U)
#define UI_DIM          APP_LCD_RGB565(128U, 128U, 128U)
#define UI_HEADER       APP_LCD_RGB565(0U, 200U, 255U)
#define UI_GREEN        APP_LCD_RGB565(0U, 220U, 0U)
#define UI_YELLOW       APP_LCD_RGB565(255U, 220U, 0U)
#define UI_ORANGE       APP_LCD_RGB565(255U, 140U, 0U)
#define UI_RED          APP_LCD_RGB565(255U, 40U, 40U)
#define UI_LINE         APP_LCD_RGB565(60U, 60U, 60U)
#define UI_TAB          APP_LCD_RGB565(40U, 40U, 40U)
#define UI_BUTTON       APP_LCD_RGB565(45U, 55U, 75U)
#define UI_BUTTON_DOWN  APP_LCD_RGB565(95U, 115U, 160U)
#define UI_BUTTON_GO    APP_LCD_RGB565(0U, 110U, 60U)

/* Navigation bar (tabs, controller status, heartbeat) on top of every page;
 * the page below it starts at UI_PAGE_Y. */
#define UI_NAV_H         48U
#define UI_PAGE_Y        (UI_NAV_H + 1U)
#define UI_MARGIN_X      16U
#define UI_STATUS_X      636U
#define UI_STATUS_Y      12U
#define UI_HEARTBEAT_X   (APP_LCD_WIDTH - 24U)

/* Home: arm controls, action result, diagnostics and the joint table. */
#define UI_LINE1_Y       130U
#define UI_LINE2_Y       150U
#define UI_RULE1_Y       170U
#define UI_TABLE_Y       176U
#define UI_ROW_PITCH     26U
#define UI_TCP_Y         416U
#define UI_RULE2_Y       438U
#define UI_FOOTER_Y      448U

/* Network page: current settings on the left, the address editor under
 * them, a keypad on the right. */
#define UI_NET_INFO_Y      60U
#define UI_NET_INFO_PITCH  32U
#define UI_NET_EDIT_Y      232U
#define UI_NET_FIELD_Y     258U
#define UI_NET_FIELD_PITCH 40U
#define UI_NET_FIELD_W     410U
#define UI_NET_MSG_Y       392U
#define UI_KEY_X           440U
#define UI_KEY_Y           56U
#define UI_KEY_W           104U
#define UI_KEY_H           56U
#define UI_KEY_GAP         8U

/* 3D page: viewport on the left, view buttons and readouts on the right. */
#define UI_3D_X            0U
#define UI_3D_Y            UI_PAGE_Y
#define UI_3D_W            584U
#define UI_3D_H            (APP_LCD_HEIGHT - UI_PAGE_Y)
#define UI_3D_PANEL_X      592U
#define UI_3D_BTN_W        96U
#define UI_3D_BTN_H        44U
#define UI_3D_TEXT_Y       174U
#define UI_3D_TEXT_PITCH   22U
#define UI_3D_TEXT_LINES   14U
#define UI_3D_TEXT_PERIOD_MS 100U

_Static_assert(UI_3D_TEXT_Y + (UI_3D_TEXT_LINES - 1U) * UI_3D_TEXT_PITCH + 16U <= APP_LCD_HEIGHT,
               "3D readouts must fit below the view buttons");


/* Every text field on a page is a slot that remembers what it shows, so a
 * frame only redraws the characters that changed (a few digits), not the
 * whole page: drawing every glyph costs ~13 ms, a typical frame well under
 * 1 ms. */
enum
{
  UI_SLOT_STATUS,
  UI_SLOT_LINE1,
  UI_SLOT_LINE2,
  UI_SLOT_TABLE_HEADER,
  UI_SLOT_JOINT0,
  UI_SLOT_TCP = UI_SLOT_JOINT0 + DM_MOTOR_COUNT,
  UI_SLOT_FOOTER,
  UI_SLOT_NET_INFO0,
  UI_SLOT_NET_EDIT_TITLE = UI_SLOT_NET_INFO0 + 5U,
  UI_SLOT_NET_FIELD0,
  UI_SLOT_NET_MSG0 = UI_SLOT_NET_FIELD0 + 3U,
  UI_SLOT_NET_MSG1,
  UI_SLOT_3D_TEXT0,
  UI_SLOT_GRAV0 = UI_SLOT_3D_TEXT0 + UI_3D_TEXT_LINES,
  UI_SLOT_LOCAL = UI_SLOT_GRAV0 + 12U,
  UI_SLOT_SLIDER0,
  UI_SLOT_DETAIL0 = UI_SLOT_SLIDER0 + APP_GRAVITY_PARAMETERS,
  UI_SLOT_COUNT = UI_SLOT_DETAIL0 + 14U
};
#define UI_SLOT_CHARS  100U

typedef struct
{
  char text[UI_SLOT_CHARS];
  uint8_t length;
  uint16_t fg;
  uint16_t x;
  uint16_t y;
  uint8_t size;
  uint8_t valid;
} UiSlot;

typedef struct
{
  uint8_t touched;
  uint8_t points;
  uint16_t x;
  uint16_t y;
  uint32_t errors;
  uint8_t dragging;   /* finger went down in the 3D viewport */
  uint16_t drag_x;
  uint16_t drag_y;
} UiTouch;

typedef enum
{
  UI_PAGE_STATUS,
  UI_PAGE_NETWORK,
  UI_PAGE_3D,
  UI_PAGE_GRAVITY,
  UI_PAGE_INFO,
  UI_PAGE_IMPEDANCE,
  UI_PAGE_ADMITTANCE,
  UI_PAGE_COUNT
} UiPage;

typedef enum
{
  UI_ACT_TAB_STATUS,
  UI_ACT_TAB_NETWORK,
  UI_ACT_TAB_3D,
  UI_ACT_KEY,        /* types the label's character */
  UI_ACT_DEL,
  UI_ACT_CLR,
  UI_ACT_REVERT,
  UI_ACT_DEFAULT,
  UI_ACT_SAVE,
  UI_ACT_VIEW_HOME,
  UI_ACT_VIEW_FRONT,
  UI_ACT_VIEW_SIDE,
  UI_ACT_VIEW_TOP,
  UI_ACT_TAB_GRAVITY, UI_ACT_GRAV_FREE, UI_ACT_GRAV_LOCK, UI_ACT_GRAV_STOP,
  UI_ACT_GRAV_JOINT_IMP, UI_ACT_GRAV_CART_IMP,
  UI_ACT_ENABLE, UI_ACT_DISABLE, UI_ACT_STOP, UI_ACT_RESUME, UI_ACT_TAB_INFO, UI_ACT_TAB_IMPEDANCE, UI_ACT_TAB_ADMITTANCE, UI_ACT_ADM_JOINT,UI_ACT_ADM_CART,UI_ACT_ADM_BIAS
} UiAction;

typedef struct
{
  uint16_t x, y, w, h;
  const char *label;
  uint8_t page;      /* UI_PAGE_COUNT: on every page (navigation bar) */
  uint8_t action;
  uint16_t color;
} UiButton;

#define UI_KEY(col, row) (uint16_t)(UI_KEY_X + (col) * (UI_KEY_W + UI_KEY_GAP)), \
                         (uint16_t)(UI_KEY_Y + (row) * (UI_KEY_H + UI_KEY_GAP)), UI_KEY_W, UI_KEY_H

static const UiButton ui_buttons_large[] = {
  {8U, 4U, 84U, 40U, "HOME", UI_PAGE_COUNT, UI_ACT_TAB_STATUS, UI_TAB},
  {100U, 4U, 96U, 40U, "NETWORK", UI_PAGE_COUNT, UI_ACT_TAB_NETWORK, UI_TAB},
  {204U, 4U, 48U, 40U, "3D", UI_PAGE_COUNT, UI_ACT_TAB_3D, UI_TAB},
  {260U, 4U, 100U, 40U, "GRAVITY", UI_PAGE_COUNT, UI_ACT_TAB_GRAVITY, UI_TAB},
  {16U, 56U, 184U, 44U, "ENABLE ARM", UI_PAGE_STATUS, UI_ACT_ENABLE, UI_BUTTON_GO},
  {212U, 56U, 184U, 44U, "DISABLE ARM", UI_PAGE_STATUS, UI_ACT_DISABLE, UI_BUTTON},
  {408U, 56U, 184U, 44U, "STOP", UI_PAGE_STATUS, UI_ACT_STOP, APP_LCD_RGB565(150U,20U,20U)},
  {604U, 56U, 184U, 44U, "RESUME", UI_PAGE_STATUS, UI_ACT_RESUME, UI_BUTTON_GO},
  {16U, 100U, 220U, 44U, "FREE", UI_PAGE_GRAVITY, UI_ACT_GRAV_FREE, UI_BUTTON},
  {252U, 100U, 220U, 44U, "LOCK", UI_PAGE_GRAVITY, UI_ACT_GRAV_LOCK, UI_BUTTON},
  {488U, 100U, 220U, 44U, "STOP/HOLD", UI_PAGE_GRAVITY, UI_ACT_GRAV_STOP, APP_LCD_RGB565(150U,20U,20U)},
#define UI_3D_BTN(col, row) (uint16_t)(UI_3D_PANEL_X + (col) * (UI_3D_BTN_W + 8U)), \
                            (uint16_t)(UI_PAGE_Y + 8U + (row) * (UI_3D_BTN_H + 8U)), UI_3D_BTN_W, UI_3D_BTN_H
  {UI_3D_BTN(0, 0), "HOME", UI_PAGE_3D, UI_ACT_VIEW_HOME, UI_BUTTON},
  {UI_3D_BTN(1, 0), "FRONT", UI_PAGE_3D, UI_ACT_VIEW_FRONT, UI_BUTTON},
  {UI_3D_BTN(0, 1), "SIDE", UI_PAGE_3D, UI_ACT_VIEW_SIDE, UI_BUTTON},
  {UI_3D_BTN(1, 1), "TOP", UI_PAGE_3D, UI_ACT_VIEW_TOP, UI_BUTTON},
  {UI_KEY(0, 0), "1", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(1, 0), "2", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(2, 0), "3", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(0, 1), "4", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(1, 1), "5", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(2, 1), "6", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(0, 2), "7", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(1, 2), "8", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(2, 2), "9", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(0, 3), ".", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(1, 3), "0", UI_PAGE_NETWORK, UI_ACT_KEY, UI_BUTTON},
  {UI_KEY(2, 3), "DEL", UI_PAGE_NETWORK, UI_ACT_DEL, UI_BUTTON},
  {UI_KEY(0, 4), "CLR", UI_PAGE_NETWORK, UI_ACT_CLR, UI_BUTTON},
  {UI_KEY(1, 4), "REVERT", UI_PAGE_NETWORK, UI_ACT_REVERT, UI_BUTTON},
  {UI_KEY(2, 4), "DEFAULT", UI_PAGE_NETWORK, UI_ACT_DEFAULT, UI_BUTTON},
  {UI_KEY_X, (uint16_t)(UI_KEY_Y + 5U * (UI_KEY_H + UI_KEY_GAP)),
   (uint16_t)(3U * UI_KEY_W + 2U * UI_KEY_GAP), UI_KEY_H, "SAVE & APPLY", UI_PAGE_NETWORK,
   UI_ACT_SAVE, UI_BUTTON_GO},
  {464U,4U,68U,40U,"INFO",UI_PAGE_COUNT,UI_ACT_TAB_INFO,UI_TAB},
  {16,100,220,44,"JOINT IMP",UI_PAGE_IMPEDANCE,UI_ACT_GRAV_JOINT_IMP,UI_BUTTON},
  {252,100,220,44,"CART IMP",UI_PAGE_IMPEDANCE,UI_ACT_GRAV_CART_IMP,UI_BUTTON},
  {368,4,88,40,"IMPED",UI_PAGE_COUNT,UI_ACT_TAB_IMPEDANCE,UI_TAB},
  {540,4,88,40,"ADMIT",UI_PAGE_COUNT,UI_ACT_TAB_ADMITTANCE,UI_TAB},
  {16,100,220,44,"JOINT ADM",UI_PAGE_ADMITTANCE,UI_ACT_ADM_JOINT,UI_BUTTON},
  {252,100,220,44,"CART ADM",UI_PAGE_ADMITTANCE,UI_ACT_ADM_CART,UI_BUTTON},
  {488,100,220,44,"STOP / HOLD",UI_PAGE_ADMITTANCE,UI_ACT_GRAV_STOP,APP_LCD_RGB565(150U,20U,20U)},
  {560,422,224,42,"Bias learn",UI_PAGE_ADMITTANCE,UI_ACT_ADM_BIAS,UI_BUTTON},
  {488,100,220,44,"STOP / HOLD",UI_PAGE_IMPEDANCE,UI_ACT_GRAV_STOP,APP_LCD_RGB565(150U,20U,20U)},
};
/* The 4.3in panel has its own layout (app_lcd_ui_small.inc); everything
 * else in this file, the tables, pages and touch handling, is the 5in one. */
#define UI_SMALL (AppLcd_PanelModel()==APP_LCD_PANEL_43)
#define ui_buttons ui_buttons_large
#define UI_BUTTON_COUNT (sizeof(ui_buttons_large) / sizeof(ui_buttons_large[0]))
#define UI_NO_BUTTON    0xFFU

/* Address editor: IP, netmask and gateway as dotted-quad text. */
#define UI_FIELD_CHARS  15U
static const char *const ui_field_names[3] = {"IP", "MASK", "GATEWAY"};

typedef struct
{
  char field[3][UI_FIELD_CHARS + 1U];
  uint8_t selected;
  uint8_t loaded;
  char message[2][56];
  uint16_t message_color;
} UiNetEdit;

static UiSlot ui_slots[UI_SLOT_COUNT];
static UiTouch ui_touch;
static UiPage ui_page = UI_PAGE_STATUS;
/* View of the pages with several (INFO, IMPED, ADMIT): tap the active tab. */
static uint8_t ui_detail[UI_PAGE_COUNT];
static uint8_t ui_pressed = UI_NO_BUTTON;
static AppLcdSliderTouch ui_slider = {APP_LCD_SLIDER_NONE};
static uint16_t ui_slider_drawn[APP_GRAVITY_PARAMETERS];
static uint8_t ui_adm_indicator[2]={UINT8_MAX,UINT8_MAX};
static AppLcdSliderTouch ui_adm_slider={APP_LCD_SLIDER_NONE};
static uint16_t ui_adm_drawn[2]={UINT16_MAX,UINT16_MAX};
static uint8_t ui_imp_indicator[2]={UINT8_MAX,UINT8_MAX};
static AppLcdSliderTouch ui_imp_slider={APP_LCD_SLIDER_NONE};
static uint16_t ui_imp_drawn[2]={UINT16_MAX,UINT16_MAX};
static const AppLcdSlider ui_sliders_large[APP_GRAVITY_PARAMETERS] = {
  {40, 202, 480, 48, .05f}, {40, 254, 480, 48, .5f}
};
#define ui_sliders ui_sliders_large
static UiNetEdit ui_net;
static uint32_t ui_frame_us;
static uint32_t ui_frame_max_us;
/* 3D page: time of the last frame that was actually rendered. */
static uint32_t ui_3d_us;
static uint32_t ui_3d_max_us;
static TickType_t ui_3d_text_tick;

/* One-second windows, separate UI service rate from actual 3D redraws.
 * All timestamps use unsigned DWT differences so cycle-counter wrap is
 * harmless.  Maxima include touch, page entry and time preempted by control. */
typedef struct
{
  uint32_t start_cycles, previous_start_cycles;
  uint32_t frames, draws, max_us, interval_min_us, interval_max_us, over;
  uint8_t started, have_previous;
} UiFrameWindow;

typedef struct
{
  uint32_t ui_hz10, draw_hz10, last_us, max_us, interval_min_us, interval_max_us, over;
} UiFrameReport;
static UiFrameReport ui_3d_detail_report;

static UiFrameWindow ui_window;
static UiFrameReport ui_report;
static uint8_t ui_report_pending;

static void AppLcdUi_RecordFrame(uint32_t start, uint32_t end, uint8_t rendered)
{
  const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  const uint32_t duration = end - start;
  ui_frame_us = duration / cycles_per_us;
  if (ui_frame_us > ui_frame_max_us) ui_frame_max_us = ui_frame_us;
  if (!ui_window.started)
  {
    ui_window.started = 1U;
    /* Anchor rates at the first completion.  Counting that first frame
     * without its preceding interval would overstate the initial Hz. */
    ui_window.start_cycles = end;
  }
  else
  {
    ++ui_window.frames;
    ui_window.draws += rendered;
  }
  if (ui_window.have_previous)
  {
    const uint32_t interval = (start - ui_window.previous_start_cycles) / cycles_per_us;
    if (ui_window.interval_min_us == 0U || interval < ui_window.interval_min_us)
      ui_window.interval_min_us = interval;
    if (interval > ui_window.interval_max_us) ui_window.interval_max_us = interval;
  }
  ui_window.previous_start_cycles = start;
  ui_window.have_previous = 1U;
  if (ui_frame_us > ui_window.max_us) ui_window.max_us = ui_frame_us;
  if (duration > SystemCoreClock / DM_LCD_FRAME_RATE_HZ) ++ui_window.over;
  const uint32_t elapsed = end - ui_window.start_cycles;
  if (elapsed < SystemCoreClock) return;

  ui_report.ui_hz10 = (uint32_t)((uint64_t)ui_window.frames * 10U * SystemCoreClock / elapsed);
  ui_report.draw_hz10 = (uint32_t)((uint64_t)ui_window.draws * 10U * SystemCoreClock / elapsed);
  ui_report.last_us = ui_frame_us;
  ui_report.max_us = ui_window.max_us;
  ui_report.interval_min_us = ui_window.interval_min_us;
  ui_report.interval_max_us = ui_window.interval_max_us;
  ui_report.over = ui_window.over;
  ui_report_pending = 1U;
  ui_window.start_cycles = end;
  ui_window.frames = ui_window.draws = ui_window.max_us = 0U;
  ui_window.interval_min_us = ui_window.interval_max_us = ui_window.over = 0U;
}

static const char *AppLcdUi_StopReasonName(uint8_t reason)
{
  static const char *const names[] = {
      "none", "STREAM_LOST", "MOVE_LOST", "MOVE_FAILED", "REQUESTED",
      "NOT_FOLLOWING", "INVALID_TARGET", "FEEDBACK_LOST", "DRIVE_FAULT", "TORQUE_LIMIT"};
  return reason < (sizeof(names) / sizeof(names[0])) ? names[reason] : "UNKNOWN";
}

static uint16_t AppLcdUi_StatusColor(uint8_t status)
{
  switch (status)
  {
    case ROBOT_STATUS_RT_JOINT:
    case ROBOT_STATUS_RT_CARTESIAN:
    case ROBOT_STATUS_NRT_JOINT:
    case ROBOT_STATUS_GRAVITY_FREE:
    case ROBOT_STATUS_GRAVITY_LOCK:
    case ROBOT_STATUS_JOINT_ADMITTANCE:
    case ROBOT_STATUS_CARTESIAN_ADMITTANCE:
    case ROBOT_STATUS_JOINT_IMPEDANCE:
    case ROBOT_STATUS_CARTESIAN_IMPEDANCE:
    case ROBOT_STATUS_NRT_CARTESIAN: return UI_GREEN;
    case ROBOT_STATUS_IDLE_HOLD:
    case ROBOT_STATUS_ENABLING:      return UI_YELLOW;
    case ROBOT_STATUS_STOPPING:
    case ROBOT_STATUS_RECOVERY:      return UI_ORANGE;
    case ROBOT_STATUS_FAULT:
    case ROBOT_STATUS_ESTOP:         return UI_RED;
    default:                         return UI_DIM;
  }
}

/* newlib-nano has no float printf. Torque readouts use three decimals. */
static void AppLcdUi_Fixed(char *out, size_t size, float value, uint8_t decimals)
{
  if (decimals > 3U) decimals = 3U;
  const long scale = decimals == 3U ? 1000L : (decimals == 2U ? 100L : 10L);
  const long scaled = (long)(value * (float)scale + ((value < 0.0f) ? -0.5f : 0.5f));
  snprintf(out, size, "%s%ld.%0*ld", (scaled < 0) ? "-" : "", labs(scaled) / scale,
           (int)decimals, labs(scaled) % scale);
}

static void AppLcdUi_FormatIp(char *out, size_t size, const uint8_t ip[4])
{
  snprintf(out, size, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

/* Strict dotted quad: four 1-3 digit parts, each <= 255. */
static uint8_t AppLcdUi_ParseIp(const char *text, uint8_t out[4])
{
  uint8_t part = 0U, digits = 0U;
  uint16_t value = 0U;

  for (const char *c = text;; ++c)
  {
    if (*c >= '0' && *c <= '9')
    {
      value = (uint16_t)(value * 10U + (uint16_t)(*c - '0'));
      if (++digits > 3U || value > 255U) return 0U;
    }
    else if (*c == '.' || *c == '\0')
    {
      if (digits == 0U || part > 3U) return 0U;
      out[part++] = (uint8_t)value;
      value = 0U;
      digits = 0U;
      if (*c == '\0') return (part == 4U) ? 1U : 0U;
    }
    else
    {
      return 0U;
    }
  }
}

/* Draws `text` into a slot, touching only the characters that differ from
 * what the slot shows; a colour change or an invalidated slot redraws all.
 * Characters past the new end are blanked. */
static void AppLcdUi_Text(uint8_t slot, uint16_t x, uint16_t y, const char *text, uint8_t size,
                          uint16_t fg)
{
  UiSlot *const s = &ui_slots[slot];
  const uint16_t advance = (size == 24U) ? 12U : 8U;
  size_t length = strlen(text);
  const uint8_t full = !s->valid || s->fg != fg || s->x != x || s->y != y || s->size != size;

  if (length >= UI_SLOT_CHARS) length = UI_SLOT_CHARS - 1U;
  const uint16_t screen_width=AppLcd_PhysicalWidth();
  const size_t fit=x<screen_width ? (size_t)(screen_width-x)/advance : 0U;
  if(length>fit) length=fit;
  for (size_t i = 0U; i < length; i++)
  {
    if (full || i >= s->length || s->text[i] != text[i])
      AppLcd_DrawChar((uint16_t)(x + i * advance), y, text[i], size, fg, UI_BG);
    s->text[i] = text[i];
  }
  /* Blank what the previous text left past the new end, also after a colour
   * change (IDLE_HOLD -> RT_JOINT). */
  if (s->valid && s->x == x && s->y == y && s->size == size)
    for (size_t i = length; i < s->length; i++)
      if (s->text[i] != ' ') AppLcd_DrawChar((uint16_t)(x + i * advance), y, ' ', size, fg, UI_BG);
  s->text[length] = '\0';
  s->length = (uint8_t)length;
  s->fg = fg;
  s->x = x;
  s->y = y;
  s->size = size;
  s->valid = 1U;
}

/* ---- Navigation and buttons ------------------------------------------- */

/* Confirmation comes from the control task, never from a button's pressed
 * state. A rejected switch must not unhighlight the mode still running. */
static uint8_t ImpedanceIndicator(const AppGravityStatus *g,const AppLocalControlStatus *c,unsigned index)
{
  const uint8_t mode=index?DM_ETH_CONTROL_CARTESIAN_IMPEDANCE:DM_ETH_CONTROL_JOINT_IMPEDANCE;
  if(!g->recovering && g->mode==mode) return g->entry_permille>=1000U?3U:2U;
  const uint8_t action=index?APP_LOCAL_CART_IMP:APP_LOCAL_JOINT_IMP;
  return (c->pending_actions&(1U<<action))?1U:0U;
}
static uint8_t AdmittanceIndicator(const AppGravityStatus *g,const AppLocalControlStatus *c,unsigned i){
  uint8_t mode=i?DM_ETH_CONTROL_CARTESIAN_ADMITTANCE:DM_ETH_CONTROL_JOINT_ADMITTANCE;
  if(!g->recovering && g->mode==mode)return g->entry_permille>=1000?3:2;
  return c->pending_actions&(1U<<(i?APP_LOCAL_CART_ADM:APP_LOCAL_JOINT_ADM))?1:0;
}
static void AppLcdUi_DrawButton(uint8_t index, uint8_t pressed)
{
  const UiButton *const b = &ui_buttons[index];
  const uint8_t active_tab = (b->action == UI_ACT_TAB_STATUS && ui_page == UI_PAGE_STATUS) ||
                             (b->action == UI_ACT_TAB_NETWORK && ui_page == UI_PAGE_NETWORK) ||
                             (b->action == UI_ACT_TAB_3D && ui_page == UI_PAGE_3D) ||
                             (b->action == UI_ACT_TAB_GRAVITY && ui_page == UI_PAGE_GRAVITY) ||
                             (b->action == UI_ACT_TAB_INFO && ui_page == UI_PAGE_INFO) ||
                             (b->action == UI_ACT_TAB_IMPEDANCE && ui_page == UI_PAGE_IMPEDANCE) ||
                             (b->action == UI_ACT_TAB_ADMITTANCE && ui_page == UI_PAGE_ADMITTANCE);
  uint16_t bg = pressed ? UI_BUTTON_DOWN : (active_tab ? UI_HEADER : b->color);
  const uint16_t fg = (active_tab && !pressed) ? UI_BLACK : UI_TEXT;
  const uint8_t font=(b->action==UI_ACT_TAB_INFO || b->action==UI_ACT_TAB_IMPEDANCE || b->action==UI_ACT_TAB_ADMITTANCE) ? 16U : 24U;
  char tab_label[20];
  const char *label=b->label;
  if(b->action==UI_ACT_ADM_BIAS)label=AppAdmittance_BiasLearning()?"Bias learn: ON":"Bias learn: OFF";
  if((ui_page==UI_PAGE_INFO || ui_page==UI_PAGE_IMPEDANCE || ui_page==UI_PAGE_ADMITTANCE) && active_tab) {
    snprintf(tab_label,sizeof(tab_label),"%s %u/%u",b->label,(unsigned)ui_detail[ui_page]+1U,ui_page==UI_PAGE_INFO?3U:ui_page==UI_PAGE_IMPEDANCE?5U:ui_page==UI_PAGE_ADMITTANCE?7U:2U);
    label=tab_label;
  }
  if(b->action==UI_ACT_GRAV_JOINT_IMP || b->action==UI_ACT_GRAV_CART_IMP) {
    const unsigned i=b->action==UI_ACT_GRAV_CART_IMP;
    AppGravityStatus g;AppGravity_Status(&g);
    AppLocalControlStatus c;AppLocalControl_Status(&c);
    const uint8_t indicator=ImpedanceIndicator(&g,&c,i);
    ui_imp_indicator[i]=indicator;
    if(indicator) {
      bg=indicator==3U?UI_BUTTON_GO:APP_LCD_RGB565(145U,90U,0U);
      snprintf(tab_label,sizeof(tab_label),"%s %s",i?"CART":"JOINT",
               indicator==3U?"ACTIVE":indicator==2U?"RAMP":"WAIT");
      label=tab_label;
    }
  }
  if(b->action==UI_ACT_ADM_JOINT || b->action==UI_ACT_ADM_CART){
    unsigned which=b->action==UI_ACT_ADM_CART;AppGravityStatus g;AppGravity_Status(&g);
    AppLocalControlStatus c;AppLocalControl_Status(&c);
    uint8_t indicator=AdmittanceIndicator(&g,&c,which);ui_adm_indicator[which]=indicator;
    if(indicator){
      bg=indicator==3?UI_BUTTON_GO:APP_LCD_RGB565(145U,90U,0U);
      snprintf(tab_label,sizeof(tab_label),"%s %s",which?"CART":"JOINT",indicator==3?"ACTIVE":indicator==2?"RAMP":"WAIT");label=tab_label;
    }
  }
  const uint16_t text_w = (uint16_t)(strlen(label) * (font/2U));

  AppLcd_FillRect(b->x, b->y, b->w, b->h, bg);
  AppLcd_DrawText((uint16_t)(b->x + ((b->w > text_w) ? (b->w - text_w) / 2U : 0U)),
                  (uint16_t)(b->y + (b->h - font) / 2U), label, font, fg, bg);
}

static uint8_t AppLcdUi_ButtonVisible(uint8_t index)
{
  if(ui_buttons[index].action==UI_ACT_ADM_BIAS)return ui_page==UI_PAGE_ADMITTANCE && ui_detail[ui_page]==3;
  if(ui_buttons[index].page==UI_PAGE_COUNT) return 1U;
  return ui_buttons[index].page==ui_page;
}

static uint8_t AppLcdUi_HitButton(uint16_t x, uint16_t y)
{
  for (uint8_t i = 0U; i < UI_BUTTON_COUNT; i++)
  {
    const UiButton *const b = &ui_buttons[i];
    if (AppLcdUi_ButtonVisible(i) && x >= b->x && x < b->x + b->w && y >= b->y && y < b->y + b->h)
      return i;
  }
  return UI_NO_BUTTON;
}

/* Clears the page area, forgets what its slots showed and draws the static
 * parts of the current page. */
static void SmallUi_EnterPage(void);
static void SmallUi_NetMessagesNow(void);
static void AppLcdUi_EnterPage(void)
{
  if (UI_SMALL) { SmallUi_EnterPage(); return; }
  ui_adm_indicator[0]=ui_adm_indicator[1]=UINT8_MAX;ui_adm_slider.index=APP_LCD_SLIDER_NONE;
  ui_adm_drawn[0]=ui_adm_drawn[1]=UINT16_MAX;
  ui_imp_indicator[0]=ui_imp_indicator[1]=UINT8_MAX;
  ui_imp_slider.index=APP_LCD_SLIDER_NONE;
  ui_imp_drawn[0]=ui_imp_drawn[1]=UINT16_MAX;
  ui_slider.index = APP_LCD_SLIDER_NONE;
  for (unsigned i=0; i<APP_GRAVITY_PARAMETERS; ++i) ui_slider_drawn[i]=UINT16_MAX;
  static uint8_t previous_page=UI_PAGE_COUNT;
  if(previous_page!=ui_page) {
    ui_window = (UiFrameWindow){0};
    ui_report = (UiFrameReport){0};
    ui_report_pending = 0U;
    ui_frame_us = ui_frame_max_us = ui_3d_us = ui_3d_max_us = 0U;
  }
  previous_page=ui_page;
  AppLcd_FillRect(0U,UI_PAGE_Y,AppLcd_PhysicalWidth(),AppLcd_PhysicalHeight()-UI_PAGE_Y,UI_BG);
  for (uint8_t i = 0U; i < UI_SLOT_COUNT; i++)
    if (i != UI_SLOT_STATUS) ui_slots[i].valid = 0U;
  for (uint8_t i = 0U; i < UI_BUTTON_COUNT; i++)
    if (AppLcdUi_ButtonVisible(i)) AppLcdUi_DrawButton(i, 0U);
  AppLcd_FillRect(0U,UI_NAV_H,AppLcd_PhysicalWidth(),1U,UI_LINE);
  AppLcd3d_Invalidate();
  if (ui_page == UI_PAGE_STATUS)
  {
    AppLcd_FillRect(0U, UI_RULE1_Y, APP_LCD_WIDTH, 1U, UI_LINE);
    AppLcd_FillRect(0U, UI_RULE2_Y, APP_LCD_WIDTH, 1U, UI_LINE);
  }
}

static void AppLcdUi_DrawNav(uint8_t heartbeat)
{
  const uint8_t status = RobotController_OperationalStatus();
  AppLcdUi_Text(UI_SLOT_STATUS, UI_STATUS_X, UI_STATUS_Y, RobotController_StatusName(status), 16U,
                AppLcdUi_StatusColor(status));
  AppLcd_FillRect(UI_HEARTBEAT_X, UI_STATUS_Y + 4U, 16U, 16U, heartbeat ? UI_GREEN : UI_BG);
}

/* ---- Status page ------------------------------------------------------- */

static void AppLcdUi_DrawRates(const AppTasksStatusSnapshot *stat)
{
  char line[112];
  InfrabotSafetyStatus safety = {0};

  const char *can_flag = stat->can_bus_off ? " BUS-OFF" : (stat->can_error_passive ? " PASSIVE" : "");
  snprintf(line, sizeof(line),
           "CTRL %4luHz miss %-3lu pmax %4luus | CAN tx %4lu rx %4lu txerr %-2lu TEC %-3u ev %lu%s",
           (unsigned long)stat->control_hz, (unsigned long)stat->control_missed,
           (unsigned long)stat->period_max_us, (unsigned long)stat->can_tx,
           (unsigned long)stat->can_rx, (unsigned long)stat->can_tx_errors,
           (unsigned)stat->can_tec, (unsigned long)stat->can_error_events, can_flag);
  const uint8_t can_bad = (stat->can_tx_errors | stat->control_missed | stat->can_bus_off |
                           stat->can_error_passive) != 0U;
  AppLcdUi_Text(UI_SLOT_LINE1, UI_MARGIN_X, UI_LINE1_Y, line, 16U, can_bad ? UI_RED : UI_TEXT);

  taskENTER_CRITICAL();
  AppSafety_Report(&safety);
  taskEXIT_CRITICAL();
  const char *phase = (safety.stop_phase == INFRABOT_STOP_PHASE_BRAKING) ? " BRAKING" :
                      (safety.stop_phase == INFRABOT_STOP_PHASE_HOLDING) ? " HOLDING" : "";
  snprintf(line, sizeof(line), "ETH %-4s cmd %4lu/s tel %4lu/s invalid %-4lu | STOP %s%s",
           AppEthernet_LinkUp() ? "UP" : "DOWN", (unsigned long)stat->command_rx,
           (unsigned long)stat->telemetry_tx, (unsigned long)stat->command_invalid,
           AppLcdUi_StopReasonName(safety.stop_reason), phase);
  AppLcdUi_Text(UI_SLOT_LINE2, UI_MARGIN_X, UI_LINE2_Y, line, 16U,
                (safety.stop_reason != INFRABOT_STOP_NONE) ? UI_ORANGE : UI_TEXT);
}

static void AppLcdUi_DrawJoints(void)
{
  char line[80];
  char q[12], cmd[12], tq[12];
  const uint8_t torque_limit = AppSafety_TorqueLimitMask();

  snprintf(line, sizeof(line), "%-3s%-4s%9s%9s%9s%6s  %-13s",
           "J", "EN", "q deg", "cmd deg", "tq Nm", "T C", "STATE");
  AppLcdUi_Text(UI_SLOT_TABLE_HEADER, UI_MARGIN_X, UI_TABLE_Y, line, 24U, UI_HEADER);

  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
  {
    const DM_Motor_t *motor = &dm_motors[i];
    const float direction = DM_MOTOR_DIRECTION(i);
    const uint8_t enabled = dm_motor_runtime.enabled[i];
    const uint8_t state = motor->state;
    const uint16_t y = (uint16_t)(UI_TABLE_Y + (i + 1U) * UI_ROW_PITCH);
    char name[4];

    snprintf(name, sizeof(name), "J%u", (unsigned)(i + 1U));
    AppLcdUi_Fixed(q, sizeof(q), (motor->position - dm_motor_runtime.zero_offset[i]) * direction *
                   57.2957795f, 2U);
    /* The joint command the control task follows (after the MCU's slew
     * limit, before the position-velocity drive lead); only while enabled. */
    if (dm_motor_runtime.control_enabled && enabled)
      AppLcdUi_Fixed(cmd, sizeof(cmd), (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * direction *
                     57.2957795f, 2U);
    else
      snprintf(cmd, sizeof(cmd), "-");
    AppLcdUi_Fixed(tq, sizeof(tq), motor->torque * direction, 2U);

    uint16_t color = UI_TEXT;
    if (state >= 0x8U) color = UI_RED;
    else if (i < DM_ARM_MOTOR_COUNT && (torque_limit & (1U << i)) != 0U) color = UI_ORANGE;
    else if (motor->feedback_count == 0U) color = UI_DIM;
    snprintf(line, sizeof(line), "%-3s%-4s%9s%9s%9s%6u  %-13s", name, enabled ? "ON" : "--",
             q, cmd, tq, (unsigned)motor->temperature_mos,
             (motor->feedback_count == 0U) ? "NO FEEDBACK" : AppTasks_MotorStateName(state));
    AppLcdUi_Text((uint8_t)(UI_SLOT_JOINT0 + i), UI_MARGIN_X, y, line, 24U, color);
  }
}

/* Measured tool pose: forward kinematics of J1-J7 (same frame and units as
 * the RTIK "meas" line), position in mm and ZYX roll/pitch/yaw in degrees. */
static void AppLcdUi_DrawTcp(void)
{
  float q[ARM_IK_AXES];
  ArmIkPose pose;
  char x[12], y[12], z[12], roll[12], pitch[12], yaw[12], line[112];

  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
    q[i] = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
  ArmIk_Forward(q, &pose);
  const float (*r)[3] = pose.rotation;
  float sp = -r[2][0];
  if (sp > 1.0f) sp = 1.0f;
  if (sp < -1.0f) sp = -1.0f;
  AppLcdUi_Fixed(x, sizeof(x), pose.position[0] * 1000.0f, 1U);
  AppLcdUi_Fixed(y, sizeof(y), pose.position[1] * 1000.0f, 1U);
  AppLcdUi_Fixed(z, sizeof(z), pose.position[2] * 1000.0f, 1U);
  AppLcdUi_Fixed(roll, sizeof(roll), atan2f(r[2][1], r[2][2]) * 57.2957795f, 1U);
  AppLcdUi_Fixed(pitch, sizeof(pitch), asinf(sp) * 57.2957795f, 1U);
  AppLcdUi_Fixed(yaw, sizeof(yaw), atan2f(r[1][0], r[0][0]) * 57.2957795f, 1U);
  snprintf(line, sizeof(line), "TCP mm %7s %7s %7s  RPY %6s %6s %6s", x, y, z, roll, pitch, yaw);
  AppLcdUi_Text(UI_SLOT_TCP, UI_MARGIN_X, UI_TCP_Y, line, 16U, UI_HEADER);
}

static void AppLcdUi_DrawFooter(void)
{
  char line[112];
  int n;

  if (!AppTouch_Ready())
    n = snprintf(line, sizeof(line), "TOUCH: not found");
  else if (ui_touch.touched)
    n = snprintf(line, sizeof(line), "TOUCH: GT%s @0x%02X  x=%3u y=%3u  points=%u",
                 AppTouch_ProductId(), (unsigned)AppTouch_Address(), (unsigned)ui_touch.x,
                 (unsigned)ui_touch.y, (unsigned)ui_touch.points);
  else
    n = snprintf(line, sizeof(line), "TOUCH: GT%s @0x%02X  released", AppTouch_ProductId(),
                 (unsigned)AppTouch_Address());
  if (n < 0) n = 0;
  snprintf(line + n, sizeof(line) - (size_t)n, "%*si2c_err %-4lu frame %4lu/%-5luus",
           (n < 52) ? 52 - n : 1, "", (unsigned long)ui_touch.errors,
           (unsigned long)ui_frame_us, (unsigned long)ui_frame_max_us);
  AppLcdUi_Text(UI_SLOT_FOOTER, UI_MARGIN_X, UI_FOOTER_Y, line, 16U, UI_DIM);
}

static void AppLcdUi_DrawStatusPage(const AppTasksStatusSnapshot *stat)
{
  AppLocalControlStatus control;
  AppLocalControl_Status(&control);
  uint8_t responding = 0U;
  if (AppStartup_ArmWaiting(&responding))
  {
    char waiting[64];
    snprintf(waiting, sizeof(waiting), "Waiting for arm power: %u/%u motors respond, re-probing...",
             (unsigned)responding, (unsigned)DM_ARM_MOTOR_COUNT);
    AppLcdUi_Text(UI_SLOT_LOCAL,16U,106U,waiting,16U,UI_ORANGE);
  }
  else AppLcdUi_Text(UI_SLOT_LOCAL,16U,106U,AppLocalControl_Message(&control),16U,
                     control.result ? UI_ORANGE : UI_YELLOW);
  AppLcdUi_DrawRates(stat);
  AppLcdUi_DrawJoints();
  AppLcdUi_DrawTcp();
  AppLcdUi_DrawFooter();
}

/* ---- Network page ------------------------------------------------------ */

static void AppLcdUi_NetMessage(uint16_t color, const char *line0, const char *line1)
{
  snprintf(ui_net.message[0], sizeof(ui_net.message[0]), "%s", line0);
  snprintf(ui_net.message[1], sizeof(ui_net.message[1]), "%s", line1);
  ui_net.message_color = color;
}

static void AppLcdUi_NetLoad(const AppNetConfig *config)
{
  AppLcdUi_FormatIp(ui_net.field[0], sizeof(ui_net.field[0]), config->ip);
  AppLcdUi_FormatIp(ui_net.field[1], sizeof(ui_net.field[1]), config->netmask);
  AppLcdUi_FormatIp(ui_net.field[2], sizeof(ui_net.field[2]), config->gateway);
  ui_net.loaded = 1U;
}

static void AppLcdUi_NetLoadCurrent(void)
{
  AppEthernetInfo info;
  AppEthernet_GetInfo(&info);
  AppLcdUi_NetLoad(&info.address);
}

/* Changing the address cuts the PC off, so it is only allowed with every
 * drive disabled: an arm the PC is moving must not lose its command stream
 * because someone touched the screen. */
static void AppLcdUi_NetSave(void)
{
  AppNetConfig config;
  char ip[16];

  for (uint8_t i = 0U; i < 3U; i++)
  {
    uint8_t *const out = (i == 0U) ? config.ip : (i == 1U) ? config.netmask : config.gateway;
    if (AppLcdUi_ParseIp(ui_net.field[i], out) == 0U)
    {
      char text[56];
      snprintf(text, sizeof(text), "%s is not a valid a.b.c.d address", ui_field_names[i]);
      AppLcdUi_NetMessage(UI_RED, text, "");
      ui_net.selected = i;
      return;
    }
  }
  if (AppPersist_NetworkValid(&config) == 0U)
  {
    AppLcdUi_NetMessage(UI_RED, "Rejected: check mask, host part and",
                        "that the gateway is in the subnet");
    return;
  }
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    if (dm_motor_runtime.enabled[i] != 0U || dm_motor_runtime.control_enabled != 0U)
    {
      AppLcdUi_NetMessage(UI_ORANGE, "Disable all motors first:",
                          "changing the IP drops the PC link");
      return;
    }
  }

  /* The sector erase blocks this task for about a second; show why first. */
  AppLcdUi_NetMessage(UI_YELLOW, "Saving to MCU flash...", "");
  if (UI_SMALL) SmallUi_NetMessagesNow();
  else
  {
    AppLcdUi_Text(UI_SLOT_NET_MSG0, UI_MARGIN_X, UI_NET_MSG_Y, ui_net.message[0], 16U, UI_YELLOW);
    AppLcdUi_Text(UI_SLOT_NET_MSG1, UI_MARGIN_X, UI_NET_MSG_Y + 20U, ui_net.message[1], 16U, UI_YELLOW);
  }
  if (AppPersist_SaveNetwork(&config) == 0U)
  {
    AppLcdUi_NetMessage(UI_RED, "Flash save failed, address unchanged", "(see UART log)");
    return;
  }
  AppEthernet_RequestAddress(&config);
  AppLcdUi_FormatIp(ip, sizeof(ip), config.ip);
  char text[56];
  snprintf(text, sizeof(text), "Saved. Now %s", ip);
  AppLcdUi_NetMessage(UI_GREEN, text, "PC must reconnect to the new IP");
}

static void AppLcdUi_NetAction(const UiButton *button)
{
  char *const field = ui_net.field[ui_net.selected];
  const size_t length = strlen(field);

  switch (button->action)
  {
    case UI_ACT_KEY:
      if (length < UI_FIELD_CHARS)
      {
        field[length] = button->label[0];
        field[length + 1U] = '\0';
      }
      break;
    case UI_ACT_DEL:
      if (length > 0U) field[length - 1U] = '\0';
      break;
    case UI_ACT_CLR:
      field[0] = '\0';
      break;
    case UI_ACT_REVERT:
      AppLcdUi_NetLoadCurrent();
      AppLcdUi_NetMessage(UI_DIM, "Fields reset to the address in use", "");
      break;
    case UI_ACT_DEFAULT:
    {
      const AppNetConfig defaults = {
        {DM_ETH_IP_ADDR0, DM_ETH_IP_ADDR1, DM_ETH_IP_ADDR2, DM_ETH_IP_ADDR3},
        {DM_ETH_NETMASK0, DM_ETH_NETMASK1, DM_ETH_NETMASK2, DM_ETH_NETMASK3},
        {DM_ETH_GW_ADDR0, DM_ETH_GW_ADDR1, DM_ETH_GW_ADDR2, DM_ETH_GW_ADDR3}};
      AppLcdUi_NetLoad(&defaults);
      AppLcdUi_NetMessage(UI_DIM, "Factory default loaded,", "press SAVE & APPLY to use it");
      break;
    }
    case UI_ACT_SAVE:
      AppLcdUi_NetSave();
      break;
    default:
      break;
  }
}

static void AppLcdUi_DrawNetworkPage(void)
{
  AppEthernetInfo info;
  char a[16], line[64];

  AppEthernet_GetInfo(&info);
  if (!ui_net.loaded) AppLcdUi_NetLoad(&info.address);

  AppLcdUi_FormatIp(a, sizeof(a), info.address.ip);
  snprintf(line, sizeof(line), "IP      %-15s %s", a,
           info.pending ? "applying" : (info.from_flash ? "saved" : "default"));
  AppLcdUi_Text(UI_SLOT_NET_INFO0, UI_MARGIN_X, UI_NET_INFO_Y, line, 24U, UI_HEADER);
  AppLcdUi_FormatIp(a, sizeof(a), info.address.netmask);
  snprintf(line, sizeof(line), "MASK    %s", a);
  AppLcdUi_Text(UI_SLOT_NET_INFO0 + 1U, UI_MARGIN_X, UI_NET_INFO_Y + UI_NET_INFO_PITCH, line, 24U,
                UI_TEXT);
  AppLcdUi_FormatIp(a, sizeof(a), info.address.gateway);
  snprintf(line, sizeof(line), "GATEWAY %s", a);
  AppLcdUi_Text(UI_SLOT_NET_INFO0 + 2U, UI_MARGIN_X, UI_NET_INFO_Y + 2U * UI_NET_INFO_PITCH, line,
                24U, UI_TEXT);
  snprintf(line, sizeof(line), "MAC     %02X:%02X:%02X:%02X:%02X:%02X", info.mac[0], info.mac[1],
           info.mac[2], info.mac[3], info.mac[4], info.mac[5]);
  AppLcdUi_Text(UI_SLOT_NET_INFO0 + 3U, UI_MARGIN_X, UI_NET_INFO_Y + 3U * UI_NET_INFO_PITCH, line,
                24U, UI_TEXT);
  if (info.client_valid)
  {
    AppLcdUi_FormatIp(a, sizeof(a), info.client_ip);
    snprintf(line, sizeof(line), "LINK %-4s PC %s:%u", info.link_up ? "UP" : "DOWN", a,
             (unsigned)info.client_port);
  }
  else
  {
    snprintf(line, sizeof(line), "LINK %-4s PC none", info.link_up ? "UP" : "DOWN");
  }
  AppLcdUi_Text(UI_SLOT_NET_INFO0 + 4U, UI_MARGIN_X, UI_NET_INFO_Y + 4U * UI_NET_INFO_PITCH, line,
                24U, info.link_up ? UI_GREEN : UI_ORANGE);

  AppLcdUi_Text(UI_SLOT_NET_EDIT_TITLE, UI_MARGIN_X, UI_NET_EDIT_Y,
                "NEW ADDRESS - tap a field, type, SAVE", 16U, UI_DIM);
  for (uint8_t i = 0U; i < 3U; i++)
  {
    const uint8_t selected = (i == ui_net.selected);
    snprintf(line, sizeof(line), "%c %-7s %s%s", selected ? '>' : ' ', ui_field_names[i],
             ui_net.field[i], selected ? "_" : "");
    AppLcdUi_Text((uint8_t)(UI_SLOT_NET_FIELD0 + i), UI_MARGIN_X,
                  (uint16_t)(UI_NET_FIELD_Y + i * UI_NET_FIELD_PITCH), line, 24U,
                  selected ? UI_YELLOW : UI_TEXT);
  }
  AppLcdUi_Text(UI_SLOT_NET_MSG0, UI_MARGIN_X, UI_NET_MSG_Y, ui_net.message[0], 16U, ui_net.message_color);
  AppLcdUi_Text(UI_SLOT_NET_MSG1, UI_MARGIN_X, UI_NET_MSG_Y + 20U, ui_net.message[1], 16U, ui_net.message_color);
}

/* ---- 3D page ----------------------------------------------------------- */

static void AppLcdUi_3dAction(uint8_t action)
{
  switch (action)
  {
    case UI_ACT_VIEW_HOME:  AppLcd3d_SetView(APP_LCD3D_VIEW_HOME);  break;
    case UI_ACT_VIEW_FRONT: AppLcd3d_SetView(APP_LCD3D_VIEW_FRONT); break;
    case UI_ACT_VIEW_SIDE:  AppLcd3d_SetView(APP_LCD3D_VIEW_SIDE);  break;
    case UI_ACT_VIEW_TOP:   AppLcd3d_SetView(APP_LCD3D_VIEW_TOP);   break;
    default:                                                        break;
  }
}

static uint8_t AppLcdUi_Draw3dPage(void)
{
  float q[ARM_IK_AXES];
  ArmIkPose pose;
  char x[12], y[12], z[12], line[40];
  const char *lines[UI_3D_TEXT_LINES];

  const uint32_t start = DWT->CYCCNT;
  const uint8_t rendered = AppLcd3d_Render(UI_3D_X, UI_3D_Y, UI_3D_W, UI_3D_H);
  if (rendered)
  {
    ui_3d_us = (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);
    if (ui_3d_us > ui_3d_max_us) ui_3d_max_us = ui_3d_us;
  }

  /* Model and touch stay at 30 Hz.  FK for the readouts, formatting and
   * changing glyphs run at most 10 Hz, with an immediate update on entry. */
  const TickType_t now = xTaskGetTickCount();
  if (ui_slots[UI_SLOT_3D_TEXT0].valid &&
      (TickType_t)(now - ui_3d_text_tick) < pdMS_TO_TICKS(UI_3D_TEXT_PERIOD_MS)) return rendered;
  ui_3d_text_tick = now;

  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
    q[i] = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
  ArmIk_Forward(q, &pose);
  AppLcdUi_Fixed(x, sizeof(x), pose.position[0] * 1000.0f, 1U);
  AppLcdUi_Fixed(y, sizeof(y), pose.position[1] * 1000.0f, 1U);
  AppLcdUi_Fixed(z, sizeof(z), pose.position[2] * 1000.0f, 1U);

  char lx[16], ly[16], lz[16];
  snprintf(lx, sizeof(lx), "X %9s", x);
  snprintf(ly, sizeof(ly), "Y %9s", y);
  snprintf(lz, sizeof(lz), "Z %9s", z);
  snprintf(line, sizeof(line), "render %5lu/%-5luus", (unsigned long)ui_3d_us,
           (unsigned long)ui_3d_max_us);
  lines[0] = "TCP (measured) mm";
  lines[1] = lx;
  lines[2] = ly;
  lines[3] = lz;
  AppLcd3dTiming timing;
  const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  char breakdown[40], raster[40], transfer[40], area[40];
  char rates[40], full_frame[40], interval[40], over[40];
  AppLcd3d_Timing(&timing);
  snprintf(breakdown, sizeof(breakdown), "geo %5lu px %5lu us",
           (unsigned long)(timing.prepare_cycles / cycles_per_us),
           (unsigned long)(timing.draw_cycles / cycles_per_us));
  snprintf(raster, sizeof(raster), "clr %5lu ras %5lu us",
           (unsigned long)(timing.clear_cycles / cycles_per_us),
           (unsigned long)(timing.raster_cycles / cycles_per_us));
  snprintf(transfer, sizeof(transfer), "cln %5lu cpy %5lu us",
           (unsigned long)(timing.clean_cycles / cycles_per_us),
           (unsigned long)(timing.blit_cycles / cycles_per_us));
  snprintf(area, sizeof(area), "area %3lu%% %6lu px",
           (unsigned long)(timing.updated_pixels * 100U / (UI_3D_W * UI_3D_H)),
           (unsigned long)timing.updated_pixels);
  lines[4] = breakdown;
  lines[5] = raster;
  lines[6] = transfer;
  lines[7] = area;
  lines[8] = line;
  snprintf(rates, sizeof(rates), "UI %2lu.%lu draw %2lu.%luHz",
           (unsigned long)(ui_report.ui_hz10 / 10U), (unsigned long)(ui_report.ui_hz10 % 10U),
           (unsigned long)(ui_report.draw_hz10 / 10U), (unsigned long)(ui_report.draw_hz10 % 10U));
  snprintf(full_frame, sizeof(full_frame), "frame %5lu/%5luus",
           (unsigned long)ui_report.last_us, (unsigned long)ui_report.max_us);
  snprintf(interval, sizeof(interval), "gap %5lu/%5luus",
           (unsigned long)ui_report.interval_min_us, (unsigned long)ui_report.interval_max_us);
  snprintf(over, sizeof(over), "over %lu %s t%lu", (unsigned long)ui_report.over,
           timing.sram_buffer ? "SRAM" : "SDRAM", (unsigned long)timing.tiles);
  lines[9] = rates;
  lines[10] = full_frame;
  lines[11] = interval;
  lines[12] = over;
  lines[13] = "drag the view to rotate";
  for (uint8_t n = 0U; n < UI_3D_TEXT_LINES; ++n)
    AppLcdUi_Text((uint8_t)(UI_SLOT_3D_TEXT0 + n), UI_3D_PANEL_X,
                  (uint16_t)(UI_3D_TEXT_Y + n * UI_3D_TEXT_PITCH), lines[n], 16U,
                  (n == 0U) ? UI_HEADER : ((n <= 3U) ? UI_TEXT : UI_DIM));
  return rendered;
}

/* Print the previous completed window during the next UI frame so the
 * printing cost itself is included in that frame's measured work. */
static void AppLcdUi_Report3d(void)
{
  if (!ui_report_pending) return;
  ui_report_pending = 0U;
  AppLcd3dTiming timing;
  AppLcd3d_Timing(&timing);
  printf("LCD3D ui=%lu.%luHz draw=%lu.%luHz frame=%lu/%luus gap=%lu..%luus over=%lu "
         "render=%lu/%luus buf=%s area=%lu%% tiles=%lu mesh=%u\r\n",
         (unsigned long)(ui_report.ui_hz10 / 10U), (unsigned long)(ui_report.ui_hz10 % 10U),
         (unsigned long)(ui_report.draw_hz10 / 10U), (unsigned long)(ui_report.draw_hz10 % 10U),
         (unsigned long)ui_report.last_us, (unsigned long)ui_report.max_us,
         (unsigned long)ui_report.interval_min_us, (unsigned long)ui_report.interval_max_us,
         (unsigned long)ui_report.over, (unsigned long)ui_3d_us, (unsigned long)ui_3d_max_us,
         timing.sram_buffer ? "SRAM" : "SDRAM",
         (unsigned long)(timing.updated_pixels * 100U / (UI_3D_W * UI_3D_H)),
         (unsigned long)timing.tiles, (unsigned int)APP_LCD_MESH_TOTAL_TRIANGLES);
}

static const char *ComplianceState(const AppGravityStatus *g)
{
  if(g->recovering) return "STOPPING: restoring hold gains";
  if(g->mode==DM_ETH_CONTROL_JOINT_IMPEDANCE) return "JOINT IMP: fixed joint equilibrium; spring + damper";
  if(g->mode==DM_ETH_CONTROL_CARTESIAN_IMPEDANCE) return "CART IMP: fixed TCP equilibrium; spring + damper";
  if(g->mode==DM_ETH_CONTROL_GRAVITY_FREE) return "FREE: hand-guided; damping remains active";
  if(g->mode==DM_ETH_CONTROL_GRAVITY_LOCK) return "LOCK: holds pose; yields when moved";
  return "Enable arm on HOME; select local compliance mode";
}
static void AppLcdUi_DrawGravityPage(void)
{
  AppGravityStatus g;
  AppGravityParameters p;
  AppLocalControlStatus control;
  char line[100];
  AppGravity_Status(&g);
  AppGravity_GetParameters(&p);
  AppLocalControl_Status(&control);
  snprintf(line,sizeof(line),"Local gravity: %s",ArmGravity_ModelName());
  AppLcdUi_Text(UI_SLOT_GRAV0,16U,62U,line,24U,UI_HEADER);
  const char *state=ComplianceState(&g);
  AppLcdUi_Text(UI_SLOT_GRAV0+1U,16U,154U,state,16U,UI_TEXT);
  if(g.fault.sequence && !control.result) {
    char joint[12],value[20],limit[20];
    if(g.fault.joint<7U) snprintf(joint,sizeof(joint),"J%u",(unsigned)g.fault.joint+1U);
    else snprintf(joint,sizeof(joint),"arm");
    const uint8_t check=g.fault.check;
    const char *unit=check==APP_GRAVITY_CHECK_SPEED ? "rad/s" :
      (check==APP_GRAVITY_CHECK_LOWER || check==APP_GRAVITY_CHECK_UPPER) ? "rad" :
      check==APP_GRAVITY_CHECK_BUDGET ? "us" : check==APP_GRAVITY_CHECK_FEEDBACK ? "ms" :
      check==APP_GRAVITY_CHECK_TEMPERATURE ? "C" : "";
    AppLcdUi_Fixed(value,sizeof(value),g.fault.value,3U);
    AppLcdUi_Fixed(limit,sizeof(limit),g.fault.limit,3U);
    snprintf(line,sizeof(line),"Last stop %s %s: %s / %s %s",joint,
             AppGravity_CheckName(check),value,limit,unit);
    AppLcdUi_Text(UI_SLOT_GRAV0+2U,16U,176U,line,16U,UI_ORANGE);
  } else AppLcdUi_Text(UI_SLOT_GRAV0+2U,16U,176U,AppLocalControl_Message(&control),16U,
                       control.result ? UI_ORANGE : UI_DIM);
  static const char *const names[] = {"Damping Kd", "Lock Kp"};
  static const char *const ranges[] = {"0.1..3 Nm s/rad", "0.5..30 Nm/rad"};
  uint8_t changing=0U;
  for (unsigned i=0;i<APP_GRAVITY_PARAMETERS;++i) {
    const AppLcdSlider *slider=&ui_sliders[i];
    char value[16]; AppLcdUi_Fixed(value,sizeof(value),p.value[i],2U);
    snprintf(line,sizeof(line),"%s: %s  [%s]",names[i],value,ranges[i]);
    AppLcdUi_Text((uint8_t)(UI_SLOT_SLIDER0+i),slider->x,slider->y,line,16U,UI_TEXT);
    const uint16_t knob=AppLcdSlider_Position(slider,p.value[i],AppGravity_ParameterMin(i),AppGravity_ParameterMax(i));
    if (ui_slider_drawn[i]!=knob) {
      AppLcd_FillRect(slider->x-6U,slider->y+20U,slider->width+13U,26U,UI_BG);
      AppLcd_FillRect(slider->x,slider->y+30U,slider->width,6U,UI_LINE);
      AppLcd_FillRect(slider->x,slider->y+30U,knob,6U,UI_HEADER);
      AppLcd_FillRect(slider->x+knob-6U,slider->y+22U,13U,22U,UI_HEADER);
      ui_slider_drawn[i]=knob;
    }
    if (fabsf(p.value[i]-p.applied[i])>.005f) changing=1U;
  }
  AppLcdUi_Text(UI_SLOT_LOCAL,568U,202U,"FF / drive Nm",16U,UI_HEADER);
  for(unsigned i=0;i<7;++i) {
    char torque[20],feedback[20]; AppLcdUi_Fixed(torque,sizeof(torque),g.torque[i],2U);
    AppLcdUi_Fixed(feedback,sizeof(feedback),g.feedback_torque[i],2U);
    snprintf(line,sizeof(line),"J%u %s / %s",i+1,torque,feedback);
    AppLcdUi_Text((uint8_t)(UI_SLOT_GRAV0+3U+i),568U,(uint16_t)(230U+24U*i),line,16U,UI_TEXT);
  }
  if(g.speed_limited_mask) {
    char joints[24]={0}; unsigned used=0U;
    for(unsigned i=0;i<7;++i) if(g.speed_limited_mask & (1U<<i))
      used+=(unsigned)snprintf(joints+used,sizeof(joints)-used,"J%u ",i+1U);
    snprintf(line,sizeof(line),"Braking (speed/joint limit): %s",joints);
    AppLcdUi_Text(UI_SLOT_GRAV0+10U,16U,420U,line,16U,UI_ORANGE);
  } else AppLcdUi_Text(UI_SLOT_GRAV0+10U,16U,420U,
      changing ? "Applying smoothly...  RAM settings; reset on reboot." :
                 "Live sliders; RAM settings reset on reboot. STOP holds enabled.",16U,UI_YELLOW);
  snprintf(line,sizeof(line),"1000 Hz | calc %lu/%lu us | model %.16s",
           (unsigned long)g.last_us,(unsigned long)g.max_us,ArmGravity_ModelHash());
  AppLcdUi_Text(UI_SLOT_GRAV0+11U,16U,446U,line,16U,UI_DIM);
}

#include "app_lcd_ui_info.inc"
#include "app_lcd_ui_impedance.inc"
#include "app_lcd_ui_admittance.inc"
#include "app_lcd_ui_small.inc"

/* ---- Touch ------------------------------------------------------------- */

static void AppLcdUi_PollTouch(void)
{
  AppTouchPoint points[APP_TOUCH_MAX_POINTS];
  uint8_t count = 0U;
  const uint8_t result = AppTouch_Read(points, &count);

  if (result == 0xFFU)
  {
    ui_touch.errors++;
    return;
  }
  if (result == 0U) return;
  ui_touch.points = count;
  ui_touch.touched = (count != 0U) ? 1U : 0U;
  if (ui_touch.touched)
  {
    uint16_t rx=0U,ry=0U;
    if (!AppTouch_Range(&rx,&ry) ||
        !AppLcdPanel_Touch(points[0].x,points[0].y,rx,ry,AppLcd_PanelModel(),
                           &ui_touch.x,&ui_touch.y))
    {
      ui_touch.touched=0U;
      ui_touch.points=0U;
    }
  }
}

/* Acts on the finger-down edge; the pressed button stays highlighted until
 * the finger lifts.  Returns 1 when the page changed. */
static uint8_t AppLcdUi_HandleTouch(uint8_t was_touched)
{
  if (UI_SMALL && !(ui_touch.touched && was_touched && ui_touch.dragging)) return SmallUi_HandleTouch(was_touched);
  if (ui_touch.touched && was_touched && ui_touch.dragging)
  {
    AppLcd3d_Orbit((int16_t)(ui_touch.x - ui_touch.drag_x), (int16_t)(ui_touch.y - ui_touch.drag_y));
    ui_touch.drag_x = ui_touch.x;
    ui_touch.drag_y = ui_touch.y;
    return 0U;
  }
  if (!ui_touch.touched)
  {
    ui_slider.index = APP_LCD_SLIDER_NONE;
    ui_adm_slider.index=APP_LCD_SLIDER_NONE;
    ui_imp_slider.index = APP_LCD_SLIDER_NONE;
    ui_touch.dragging = 0U;
    if (ui_pressed != UI_NO_BUTTON)
    {
      if (AppLcdUi_ButtonVisible(ui_pressed)) AppLcdUi_DrawButton(ui_pressed, 0U);
      ui_pressed = UI_NO_BUTTON;
    }
    return 0U;
  }
  if(ui_page==UI_PAGE_ADMITTANCE && ui_adm_slider.index!=APP_LCD_SLIDER_NONE){AppLcdUi_AdmittanceTouch();return 0U;}
  if(ui_page==UI_PAGE_IMPEDANCE && ui_imp_slider.index!=APP_LCD_SLIDER_NONE) {
    AppLcdUi_ImpedanceTouch();return 0U;
  }
  if (ui_page==UI_PAGE_GRAVITY && ui_slider.index!=APP_LCD_SLIDER_NONE) {
    const uint8_t i=ui_slider.index;
    const float value=AppLcdSlider_Value(&ui_sliders[i],ui_touch.x,
                                        AppGravity_ParameterMin(i),AppGravity_ParameterMax(i));
    (void)AppGravity_SetParameter(i,value);
    return 0U; /* captured drag never activates another slider or button */
  }
  if (was_touched) return 0U;

  const uint8_t hit = AppLcdUi_HitButton(ui_touch.x, ui_touch.y);
  if (hit != UI_NO_BUTTON)
  {
    const UiButton *const button = &ui_buttons[hit];
    if (button->action == UI_ACT_TAB_STATUS || button->action == UI_ACT_TAB_NETWORK ||
        button->action == UI_ACT_TAB_3D || button->action == UI_ACT_TAB_GRAVITY ||
        button->action == UI_ACT_TAB_INFO || button->action == UI_ACT_TAB_IMPEDANCE || button->action==UI_ACT_TAB_ADMITTANCE)
    {
      const UiPage page = (button->action == UI_ACT_TAB_STATUS) ? UI_PAGE_STATUS :
                          (button->action == UI_ACT_TAB_NETWORK) ? UI_PAGE_NETWORK :
                          (button->action == UI_ACT_TAB_3D) ? UI_PAGE_3D :
                          (button->action == UI_ACT_TAB_GRAVITY) ? UI_PAGE_GRAVITY :
                          (button->action == UI_ACT_TAB_INFO) ? UI_PAGE_INFO : button->action==UI_ACT_TAB_ADMITTANCE?UI_PAGE_ADMITTANCE:UI_PAGE_IMPEDANCE;
      if (page == ui_page) {
        if(page!=UI_PAGE_INFO && page!=UI_PAGE_IMPEDANCE && page!=UI_PAGE_ADMITTANCE) return 0U;
        ui_detail[page]=(uint8_t)((ui_detail[page]+1U)%(page==UI_PAGE_INFO?3U:page==UI_PAGE_IMPEDANCE?5U:7U));
        return 1U;
      }
      ui_page = page;
      ui_detail[page]=0U;
      if (page == UI_PAGE_NETWORK)
      {
        AppLcdUi_NetLoadCurrent();
        AppLcdUi_NetMessage(UI_DIM, "", "");
      }
      return 1U;
    }
    ui_pressed = hit;
    AppLcdUi_DrawButton(hit, 1U);
    if (ui_page == UI_PAGE_3D) AppLcdUi_3dAction(button->action);
    else if (ui_page == UI_PAGE_GRAVITY || ui_page == UI_PAGE_IMPEDANCE || ui_page==UI_PAGE_ADMITTANCE) {
      if(button->action==UI_ACT_GRAV_FREE) AppLocalControl_Request(APP_LOCAL_FREE);
      else if(button->action==UI_ACT_GRAV_LOCK) AppLocalControl_Request(APP_LOCAL_LOCK);
      else if(button->action==UI_ACT_GRAV_JOINT_IMP) AppLocalControl_Request(APP_LOCAL_JOINT_IMP);
      else if(button->action==UI_ACT_GRAV_CART_IMP) AppLocalControl_Request(APP_LOCAL_CART_IMP);
      else if(button->action==UI_ACT_ADM_JOINT)AppLocalControl_Request(APP_LOCAL_JOINT_ADM);
      else if(button->action==UI_ACT_ADM_BIAS)AppAdmittance_SetBiasLearning(!AppAdmittance_BiasLearning());
      else if(button->action==UI_ACT_ADM_CART)AppLocalControl_Request(APP_LOCAL_CART_ADM);
      else if(button->action==UI_ACT_GRAV_STOP) AppLocalControl_Request(APP_LOCAL_STOP);
    }
    else if (ui_page==UI_PAGE_STATUS) {
      if (button->action==UI_ACT_ENABLE) AppLocalControl_Request(APP_LOCAL_ENABLE);
      else if (button->action==UI_ACT_DISABLE) AppLocalControl_Request(APP_LOCAL_DISABLE);
      else if (button->action==UI_ACT_STOP) AppLocalControl_Request(APP_LOCAL_STOP);
      else if (button->action==UI_ACT_RESUME) AppLocalControl_Request(APP_LOCAL_RESUME);
    }
    else AppLcdUi_NetAction(button);
    return 0U;
  }
  if(ui_page==UI_PAGE_IMPEDANCE) {AppLcdUi_ImpedanceTouch();return 0U;}
  if(ui_page==UI_PAGE_ADMITTANCE) {AppLcdUi_AdmittanceTouch();return 0U;}
  if (ui_page == UI_PAGE_3D && (uint16_t)(ui_touch.x - UI_3D_X) < UI_3D_W && ui_touch.y >= UI_3D_Y)
  {
    ui_touch.dragging = 1U;
    ui_touch.drag_x = ui_touch.x;
    ui_touch.drag_y = ui_touch.y;
    return 0U;
  }
  if (ui_page==UI_PAGE_GRAVITY) {
    AppLcdSlider_Capture(&ui_slider,ui_sliders,APP_GRAVITY_PARAMETERS,ui_touch.x,ui_touch.y);
    if (ui_slider.index!=APP_LCD_SLIDER_NONE) {
      const uint8_t i=ui_slider.index;
      (void)AppGravity_SetParameter(i,AppLcdSlider_Value(&ui_sliders[i],ui_touch.x,
                                      AppGravity_ParameterMin(i),AppGravity_ParameterMax(i)));
    }
    return 0U;
  }
  /* A tap on an address field selects it. */
  if (ui_page == UI_PAGE_NETWORK && ui_touch.x < UI_MARGIN_X + UI_NET_FIELD_W)
  {
    for (uint8_t i = 0U; i < 3U; i++)
    {
      const uint16_t top = (uint16_t)(UI_NET_FIELD_Y + i * UI_NET_FIELD_PITCH - 8U);
      if (ui_touch.y >= top && ui_touch.y < top + UI_NET_FIELD_PITCH) ui_net.selected = i;
    }
  }
  return 0U;
}

/* ---- Task -------------------------------------------------------------- */

/* Lowest priority: everything here runs only in time the control, Ethernet
 * and status tasks leave idle.  Shared state is read without locking (a
 * torn display value is harmless) except the short critical sections the
 * owners require (status snapshot, safety report, address request).  The one
 * long operation, saving the address to flash, only happens with every drive
 * disabled. */
#ifdef FW_NETBOOT_APP
#include "app_lcd_update.inc"
#endif
static void AppLcdUi_Task(void *argument)
{
  _Static_assert(DM_LCD_FRAME_RATE_HZ >= 2U && DM_LCD_FRAME_RATE_HZ <= configTICK_RATE_HZ,
                 "LCD frame rate must fit the RTOS tick rate");
  const uint32_t frames_per_heartbeat = DM_LCD_FRAME_RATE_HZ / 2U;
  uint32_t tick_fraction = 0U;
  TickType_t last_wake;
  uint32_t frames = 0U;
  uint8_t enter_page = 1U;
  AppTasksStatusSnapshot stat;

  (void)argument;
  const uint32_t boot_start = AppLcd_FirstFrameTick();
  TickType_t boot_wake=xTaskGetTickCount();
  uint32_t boot_fraction=0U,boot_frames=0U,boot_max_us=0U;
  printf("LCD: infrabot boot animation 3 s, shared frames target=30Hz\r\n");
  for (;;) {
    const uint32_t elapsed = HAL_GetTick()-boot_start;
    const uint32_t draw_start=DWT->CYCCNT;
    AppLcdBoot_Draw(elapsed);
    const uint32_t draw_us=(DWT->CYCCNT-draw_start)/(SystemCoreClock/1000000U);
    if(draw_us>boot_max_us) boot_max_us=draw_us;
    ++boot_frames;
    if (elapsed >= APP_LCD_BOOT_MS) break;
    AppLcdSchedule_Wait(&boot_wake,&boot_fraction,30U);
  }
  printf("LCD: boot animation frames=%lu render_max=%luus\r\n",
         (unsigned long)boot_frames,(unsigned long)boot_max_us);
  if (AppTouch_Init())
    printf("LCD: touch GT%s at I2C 0x%02X (I2C2 PH4/PH5, RST PI8)\r\n", AppTouch_ProductId(),
           (unsigned)AppTouch_Address());
  else
    printf("LCD: touch controller not found; status page without touch\r\n");
  AppLcd_FillRect(0U, 0U, APP_LCD_WIDTH, APP_LCD_HEIGHT, UI_BG);
  if (UI_SMALL) SmallUi_LoadLanguage();   /* the 4.3in screen's saved language */

  last_wake = xTaskGetTickCount();
  for (;;)
  {
    AppLcdSchedule_Wait(&last_wake, &tick_fraction, DM_LCD_FRAME_RATE_HZ);
    const uint32_t start = DWT->CYCCNT;
    uint8_t rendered = 0U;
    const uint8_t was_touched = ui_touch.touched;
    if (AppTouch_Ready()) AppLcdUi_PollTouch();
    frames++;

#ifdef FW_NETBOOT_APP
    if(UpdateModal(was_touched,&enter_page))continue;
#endif
    if (AppLcdUi_HandleTouch(was_touched)) enter_page = 1U;
    if (enter_page)
    {
      AppLcdUi_EnterPage();
      enter_page = 0U;
    }
    if (UI_SMALL)
      rendered = SmallUi_DrawFrame((uint8_t)((frames / frames_per_heartbeat) & 1U));
    else
    {
    AppLcdUi_DrawNav((uint8_t)((frames / frames_per_heartbeat) & 1U));
    if (ui_page == UI_PAGE_STATUS)
    {
      AppTasks_GetStatusSnapshot(&stat);
      AppLcdUi_DrawStatusPage(&stat);
    }
    else if (ui_page == UI_PAGE_NETWORK)
    {
      AppLcdUi_DrawNetworkPage();
    }
    else if(ui_page==UI_PAGE_ADMITTANCE){AppLcdUi_DrawAdmittancePage();}
    else if (ui_page == UI_PAGE_IMPEDANCE)
    {
      AppLcdUi_DrawImpedancePage();
    }
    else if (ui_page == UI_PAGE_INFO)
    {
      AppLcdUi_DrawInfoPage();
    }
    else if (ui_page == UI_PAGE_GRAVITY)
    {
      AppLcdUi_DrawGravityPage();
    }
    else
    {
      rendered = AppLcdUi_Draw3dPage();
      AppLcdUi_Report3d();
    }
    }
    AppLcdUi_RecordFrame(start, DWT->CYCCNT, rendered);
    if (frames == 1U)
      printf("LCD: status page %u Hz, full frame %lu us\r\n",
             (unsigned)DM_LCD_FRAME_RATE_HZ, (unsigned long)ui_frame_us);
  }
}

BaseType_t AppLcdUi_CreateTask(void)
{
  if (!AppLcd_Ready()) return pdPASS;
  return xTaskCreate(AppLcdUi_Task, "lcd", DM_LCD_TASK_STACK, NULL, DM_LCD_TASK_PRIORITY, NULL);
}
