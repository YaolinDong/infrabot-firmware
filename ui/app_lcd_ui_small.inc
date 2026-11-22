/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "control_stats.h"
#include "dm_runtime.h"
#include "eth_state.h"
/* 4.3in 480x272 layout (app_lcd_ui.c private).
 *
 * A main menu of tiles opens each function.  Every page shares a top bar:
 * BACK (sub-pages) or the product name (menu), the page name, the controller
 * state as a coloured chip, one health word and STOP.  STOP is on every page
 * at the far right, away from BACK.  Views of one function are a segmented
 * switch inside the page.  All text is the native 8x16 font at 1:1; in
 * Chinese (Settings / LANGUAGE) a character is a 16x16 glyph, two columns.
 *
 * Only reached when UI_SMALL; the 5in layout does not use anything here.
 * Runs in the low-priority LCD task like the rest of the UI: it reads shared
 * state and posts requests (AppLocalControl_Request), never drives motors. */

/* ---- Colours ------------------------------------------------------------------ */
#define S_BG      APP_LCD_RGB565(8U, 10U, 14U)
#define S_BAR     APP_LCD_RGB565(20U, 24U, 32U)
#define S_CARD    APP_LCD_RGB565(24U, 29U, 38U)
#define S_LINE    APP_LCD_RGB565(52U, 60U, 76U)
#define S_TEXT    APP_LCD_RGB565(232U, 236U, 240U)
#define S_DIM     APP_LCD_RGB565(128U, 138U, 152U)
#define S_ACCENT  APP_LCD_RGB565(0U, 200U, 255U)
#define S_GREEN   APP_LCD_RGB565(0U, 205U, 95U)
#define S_YELLOW  APP_LCD_RGB565(255U, 204U, 0U)
#define S_ORANGE  APP_LCD_RGB565(255U, 145U, 0U)
#define S_RED     APP_LCD_RGB565(240U, 50U, 50U)
#define S_BUTTON  APP_LCD_RGB565(44U, 54U, 74U)
#define S_PRESSED APP_LCD_RGB565(95U, 115U, 160U)
#define S_GO      APP_LCD_RGB565(0U, 120U, 70U)
#define S_START   APP_LCD_RGB565(0U, 96U, 192U)   /* idle mode button: tap to start that mode */
#define S_SELECT  APP_LCD_RGB565(0U, 88U, 120U)   /* the chosen one of a set of options */
#define S_WAIT    APP_LCD_RGB565(145U, 90U, 0U)
#define S_STOP    APP_LCD_RGB565(170U, 28U, 28U)
#define S_LIVE_BG APP_LCD_RGB565(16U, 52U, 34U)
#define S_WARN_BG APP_LCD_RGB565(60U, 34U, 0U)
#define S_BLACK   APP_LCD_RGB565(0U, 0U, 0U)

/* ---- Geometry ----------------------------------------------------------------- */
#define S_TOP_H      34U   /* top bar; its rule is the next row */
#define S_BACK_X     4U
#define S_BACK_W     44U
#define S_TITLE_X    56U   /* the longest title (20 characters) ends at 216 */
#define S_STATE_X    220U  /* state chip and health word: from here to STOP */
#define S_STOP_X     400U
#define S_STOP_W     76U
#define S_BAR_BTN_Y  4U
#define S_BAR_BTN_H  26U
#define S_ROW1_Y     40U   /* page buttons and the view switch */
#define S_ROW1_H     28U
#define S_STATUS_Y   74U   /* one status line; its colour carries the meaning */
#define S_BODY_Y     94U   /* cards */
#define S_BODY_H     174U
#define S_LIST_X     4U    /* parameter list card */
#define S_LIST_W     296U
#define S_SIDE_X     304U  /* joint torque card */
#define S_SIDE_W     172U
#define S_LIST_ROW_H 22U
#define S_LIST_COLS  34U
#define S_3D_Y       35U
#define S_3D_W       320U
#define S_3D_H       237U

/* Text slots of this layout; every page entry invalidates them all. */
#define SS_STATE   0U
#define SS_STATUS  1U
#define SS_ROW(i)  (uint8_t)(2U + (i))   /* 0..39 */
#define SS_SIDE(i) (uint8_t)(42U + (i))  /* 0..25 */
_Static_assert(42U + 26U <= UI_SLOT_COUNT, "4.3in layout text slots");
static uint16_t small_slot_bg[UI_SLOT_COUNT];

/* ---- Pages -------------------------------------------------------------------- */
typedef enum
{
  SP_MENU, SP_ARM, SP_3D, SP_GRAVITY, SP_NETWORK, SP_JOINT_IMP, SP_CART_IMP,
  SP_JOINT_ADM, SP_CART_ADM, SP_DIAG, SP_SETTINGS, SP_COUNT
} SmallPage;

static const char *const small_titles[SP_COUNT] = {
  "infrabot", "Arm & Joints", "3D View", "Gravity Compensation", "Network",
  "Joint Impedance", "Cartesian Impedance", "Joint Admittance", "Cartesian Admittance",
  "Diagnostics", "Settings"};

static const uint8_t small_view_count[SP_COUNT] = {
  [SP_3D] = 2U, [SP_GRAVITY] = 2U, [SP_NETWORK] = 2U, [SP_CART_IMP] = 2U,
  [SP_JOINT_ADM] = 3U, [SP_CART_ADM] = 4U, [SP_SETTINGS] = 4U};
static const char *const small_view_names[SP_COUNT][4] = {
  [SP_3D] = {"MODEL", "RENDER"},
  [SP_GRAVITY] = {"CONTROL", "DETAILS"},
  [SP_NETWORK] = {"EDIT", "STATUS"},
  [SP_CART_IMP] = {"PARAMS", "LIVE"},
  [SP_JOINT_ADM] = {"PARAMS", "SHARED", "LIVE"},
  [SP_CART_ADM] = {"PARAMS", "SHARED", "ELBOW", "LIVE"},
  [SP_SETTINGS] = {"LANGUAGE", "ABOUT", "MOTORS", "PD"}};

/* Menu tiles in display order: two columns, five rows. */
static const uint8_t small_menu_pages[10] = {
  SP_ARM, SP_3D, SP_GRAVITY, SP_NETWORK, SP_JOINT_IMP, SP_CART_IMP,
  SP_JOINT_ADM, SP_CART_ADM, SP_DIAG, SP_SETTINGS};
#define S_TILE_W   234U
#define S_TILE_H   42U
#define S_TILE_X(i) (uint16_t)(4U + ((i) % 2U) * (S_TILE_W + 4U))
#define S_TILE_Y(i) (uint16_t)(40U + ((i) / 2U) * (S_TILE_H + 4U))

/* The local action of each mode page's ENTER button. */
static const uint8_t small_enter_action[SP_COUNT] = {
  [SP_JOINT_IMP] = APP_LOCAL_JOINT_IMP, [SP_CART_IMP] = APP_LOCAL_CART_IMP,
  [SP_JOINT_ADM] = APP_LOCAL_JOINT_ADM, [SP_CART_ADM] = APP_LOCAL_CART_ADM};

/* ---- Touch targets ------------------------------------------------------------- */
typedef enum
{
  SA_NONE, SA_BACK, SA_STOP, SA_TILE, SA_VIEW, SA_ENABLE, SA_DISABLE, SA_RESUME,
  SA_ENTER, SA_FREE, SA_LOCK, SA_BIAS, SA_VIEW3D, SA_KEY, SA_LANGUAGE
} SmallAction;

typedef struct
{
  uint16_t x, y, w, h, bg, fg;
  uint8_t action, arg;
  char label[24];   /* UTF-8; empty: not drawn here (tiles, view switch) */
} SmallButton;

/* NETWORK / EDIT keypad: label and the shared address-editor action. */
static const char *const small_key_labels[16] = {
  "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "DEL", "CLR", "REVERT", "DEFAULT", "SAVE & APPLY"};
static const uint8_t small_key_actions[16] = {
  UI_ACT_KEY, UI_ACT_KEY, UI_ACT_KEY, UI_ACT_KEY, UI_ACT_KEY, UI_ACT_KEY, UI_ACT_KEY, UI_ACT_KEY,
  UI_ACT_KEY, UI_ACT_KEY, UI_ACT_KEY, UI_ACT_DEL, UI_ACT_CLR, UI_ACT_REVERT, UI_ACT_DEFAULT, UI_ACT_SAVE};
#define S_FIELD_Y(i) (uint16_t)(146U + (i) * 26U)   /* address fields; tap rows start 6 px higher */

#define S_BUTTONS 32U
typedef struct
{
  uint8_t page, view, fresh, previous_page;
  SmallButton buttons[S_BUTTONS];
  uint8_t count, pressed;              /* pressed: index into buttons, UINT8_MAX = none */
  uint8_t health_drawn, heartbeat_drawn;
  uint8_t indicator_drawn[2];          /* mode buttons as last drawn */
  uint8_t bias_drawn;
  uint8_t param, param_drawn;          /* selected row of the page's parameter list */
  uint16_t knob_drawn;
  uint8_t tile_drawn[10];
  uint8_t dot_drawn[DM_MOTOR_COUNT];
  uint8_t language;                    /* APP_UI_LANGUAGE_*, of this screen only */
  uint8_t language_saved;              /* what the next boot shows */
  uint8_t language_note;               /* SL_NOTE_*: outcome of the last save attempt */
} SmallUi;
static SmallUi small_ui = {.page = SP_MENU, .previous_page = SP_COUNT, .pressed = UINT8_MAX};

/* ---- Language ------------------------------------------------------------------ */
#include "app_lcd_ui_zh.inc"

/* The text to show for an English UI string: itself, or its Chinese entry.
 * Strings without an entry (numbers, model names) stay as they are. */
__attribute__((format_arg(1))) static const char *SmallUi_Tr(const char *en)
{
  static uint16_t order[SMALL_TR_COUNT];   /* table indices sorted by English */
  static uint8_t sorted;
  if (small_ui.language != APP_UI_LANGUAGE_ZH || en == NULL) return en;
  if (!sorted)
  {
    for (unsigned i = 0U; i < SMALL_TR_COUNT; ++i)
    {
      unsigned j = i;
      for (; j > 0U && strcmp(small_tr[order[j - 1U]].en, small_tr[i].en) > 0; --j) order[j] = order[j - 1U];
      order[j] = (uint16_t)i;
    }
    sorted = 1U;
  }
  unsigned lo = 0U, hi = SMALL_TR_COUNT;
  while (lo < hi)
  {
    const unsigned mid = (lo + hi) / 2U;
    const int c = strcmp(small_tr[order[mid]].en, en);
    if (c == 0) return small_tr[order[mid]].zh;
    if (c < 0) lo = mid + 1U;
    else hi = mid;
  }
  return en;
}
/* TR marks literals for tools/gen_lcd_zh_font.py, which refuses a TR string
 * without an entry; strings of other modules go through SmallUi_Tr directly. */
#define TR(text) SmallUi_Tr(text)

/* ---- Drawing helpers ------------------------------------------------------------ */
/* Filled rectangle with rounded corners: the corner pixels keep what is under
 * them, so the surface behind must already be painted. */
static void SmallUi_RoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t radius, uint16_t color)
{
  static const uint8_t r4[] = {2U, 1U, 0U, 0U};
  static const uint8_t r6[] = {4U, 2U, 1U, 1U, 0U, 0U};
  static const uint8_t r8[] = {5U, 3U, 2U, 1U, 1U, 0U, 0U, 0U};
  static const uint8_t r11[] = {8U, 5U, 4U, 3U, 2U, 1U, 1U, 1U, 0U, 0U, 0U};
  const uint8_t *inset = radius >= 11U ? r11 : radius >= 8U ? r8 : radius >= 6U ? r6 : r4;
  const uint16_t span = radius >= 11U ? 11U : radius >= 8U ? 8U : radius >= 6U ? 6U : 4U;
  if (h < 2U * span || w < 2U * span) { AppLcd_FillRect(x, y, w, h, color); return; }
  uint16_t rows = span;
  while (rows > 0U && inset[rows - 1U] == 0U) --rows;
  for (uint16_t j = 0U; j < rows; ++j)
  {
    const uint16_t i = inset[j];
    AppLcd_FillRect((uint16_t)(x + i), (uint16_t)(y + j), (uint16_t)(w - 2U * i), 1U, color);
    AppLcd_FillRect((uint16_t)(x + i), (uint16_t)(y + h - 1U - j), (uint16_t)(w - 2U * i), 1U, color);
  }
  AppLcd_FillRect(x, (uint16_t)(y + rows), w, (uint16_t)(h - 2U * rows), color);
}

/* Display width of UTF-8 text in pixels (8 per column). */
static uint16_t SmallUi_Width(const char *text)
{
  return (uint16_t)(AppLcdZh_Columns(text) * 8U);
}
/* One character at x, y; returns its width in pixels. */
static uint16_t SmallUi_Glyph(uint16_t x, uint16_t y, uint16_t code, uint16_t fg, uint16_t bg)
{
  if (code < 0x80U) { AppLcd_DrawChar(x, y, (char)code, 16U, fg, bg); return 8U; }
  AppLcd_DrawWide(x, y, code, fg, bg);
  return 16U;
}
static void SmallUi_Label(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg)
{
  for (uint16_t c; (c = AppLcdZh_Next(&text)) != 0U;)
  {
    if ((uint32_t)x + AppLcdZh_Width(c) * 8U > AppLcd_PhysicalWidth()) break;
    x = (uint16_t)(x + SmallUi_Glyph(x, y, c, fg, bg));
  }
}
static void SmallUi_Center(uint16_t x, uint16_t y, uint16_t w, const char *text, uint16_t fg, uint16_t bg)
{
  const uint16_t width = SmallUi_Width(text);
  SmallUi_Label((uint16_t)(x + (w > width ? (w - width) / 2U : 0U)), y, text, fg, bg);
}

/* Lays UTF-8 text out on `columns` 8-px columns: every column gets the code
 * of the character starting there, 0 for the right half of a wide one, ' '
 * past the end.  Too long ends in "...". */
static uint32_t small_text_cut;   /* texts shortened with "..." (host tests) */
static void SmallUi_Layout(const char *text, unsigned columns, uint16_t *cell)
{
  unsigned n = 0U;
  uint8_t cut = 0U;
  for (uint16_t c; (c = AppLcdZh_Next(&text)) != 0U;)
  {
    const unsigned w = AppLcdZh_Width(c);
    if (n + w > columns) { cut = 1U; break; }
    cell[n] = c;
    if (w == 2U) cell[n + 1U] = 0U;
    n += w;
  }
  for (unsigned i = n; i < columns; ++i) cell[i] = ' ';
  if (cut && columns >= 3U)
  {
    small_text_cut++;
    unsigned i = columns - 3U;
    if (cell[i] == 0U) cell[i - 1U] = ' ';   /* do not leave half a wide character */
    for (; i < columns; ++i) cell[i] = '.';
  }
}

/* Slot text on a surface `bg`: padded to `columns` so a shorter text clears
 * the old tail; only changed characters are drawn; too long ends in "...".
 * The slot keeps the UTF-8 of what is shown (the tests read it back). */
static void SmallUi_Text(uint8_t slot, uint16_t x, uint16_t y, unsigned columns, const char *text,
                         uint16_t fg, uint16_t bg)
{
  UiSlot *const s = &ui_slots[slot];
  uint16_t cell[64], old[64];
  if (columns > 63U) columns = 63U;
  SmallUi_Layout(text, columns, cell);
  const uint8_t full = !s->valid || s->fg != fg || s->x != x || s->y != y || s->size != 16U ||
                       s->length != columns || small_slot_bg[slot] != bg;
  if (!full) { const uint32_t cut = small_text_cut; SmallUi_Layout(s->text, columns, old); small_text_cut = cut; }
  char line[UI_SLOT_CHARS];
  unsigned used = 0U;
  for (unsigned i = 0U; i < columns; ++i)
  {
    if (cell[i] == 0U) continue;
    const unsigned w = AppLcdZh_Width(cell[i]);
    if (full || old[i] != cell[i] || (w == 2U && old[i + 1U] != 0U))
      (void)SmallUi_Glyph((uint16_t)(x + i * 8U), y, cell[i], fg, bg);
    if (used + 3U < sizeof(line)) used += AppLcdZh_Encode(cell[i], line + used);
  }
  line[used] = '\0';
  /* Trailing padding is not stored, so the slot always fits its buffer. */
  while (used > 0U && line[used - 1U] == ' ') line[--used] = '\0';
  memcpy(s->text, line, used + 1U);
  s->length = (uint8_t)columns;
  s->fg = fg;
  s->x = x;
  s->y = y;
  s->size = 16U;
  s->valid = 1U;
  small_slot_bg[slot] = bg;
}
static void SmallUi_Textf(uint8_t slot, uint16_t x, uint16_t y, unsigned columns, uint16_t fg, uint16_t bg,
                          const char *format, ...)
{
  char line[160];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  SmallUi_Text(slot, x, y, columns, line, fg, bg);
}

/* text padded with spaces to `columns` display columns, left or right aligned. */
static const char *SmallUi_Pad(char *out, size_t size, const char *text, unsigned columns, uint8_t right)
{
  const unsigned used = AppLcdZh_Columns(text);
  const unsigned pad = used < columns ? columns - used : 0U;
  if (right) snprintf(out, size, "%*s%s", (int)pad, "", text);
  else snprintf(out, size, "%s%*s", text, (int)pad, "");
  return out;
}

static void SmallUi_Card(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *title)
{
  SmallUi_RoundRect(x, y, w, h, 8U, S_CARD);
  if (title) SmallUi_Label((uint16_t)(x + 8U), (uint16_t)(y + 4U), title, S_DIM, S_CARD);
}

static void SmallUi_DrawButtonIndex(uint8_t index, uint8_t pressed)
{
  const SmallButton *b = &small_ui.buttons[index];
  if (!b->label[0]) return;
  const uint16_t bg = pressed ? S_PRESSED : b->bg;
  SmallUi_RoundRect(b->x, b->y, b->w, b->h, 6U, bg);
  SmallUi_Center(b->x, (uint16_t)(b->y + (b->h - 16U) / 2U), b->w, b->label, b->fg, bg);
}

/* Registers (or updates, same action and argument) a touch target and draws it
 * when it has a label. */
static void SmallUi_PutButton(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t action, uint8_t arg,
                              const char *label, uint16_t bg, uint16_t fg)
{
  uint8_t index = small_ui.count;
  for (uint8_t i = 0U; i < small_ui.count; ++i)
    if (small_ui.buttons[i].action == action && small_ui.buttons[i].arg == arg) { index = i; break; }
  if (index == small_ui.count)
  {
    if (small_ui.count >= S_BUTTONS) return;
    small_ui.count++;
  }
  SmallButton *b = &small_ui.buttons[index];
  *b = (SmallButton){x, y, w, h, bg, fg, action, arg, ""};
  if (label) snprintf(b->label, sizeof(b->label), "%s", label);
  SmallUi_DrawButtonIndex(index, small_ui.pressed == index);
}

static void SmallUi_Pose(char xyz[3][16], char rpy[3][16])
{
  float q[ARM_IK_AXES];
  ArmIkPose pose;
  for (unsigned i = 0U; i < ARM_IK_AXES; ++i)
    q[i] = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
  ArmIk_Forward(q, &pose);
  for (unsigned i = 0U; i < 3U; ++i) AppLcdUi_Fixed(xyz[i], 16, pose.position[i] * 1000.0f, 1U);
  const float (*r)[3] = pose.rotation;
  AppLcdUi_Fixed(rpy[0], 16, atan2f(r[2][1], r[2][2]) * 57.2957795f, 1U);
  AppLcdUi_Fixed(rpy[1], 16, asinf(fminf(1.0f, fmaxf(-1.0f, -r[2][0]))) * 57.2957795f, 1U);
  AppLcdUi_Fixed(rpy[2], 16, atan2f(r[1][0], r[0][0]) * 57.2957795f, 1U);
}

/* One status line under the page buttons. */
static void SmallUi_Status(const char *text, uint16_t color)
{
  SmallUi_Text(SS_STATUS, 6U, S_STATUS_Y, 58U, text, color, S_BG);
}

/* ---- Top bar ---------------------------------------------------------------------- */
/* Controller state for the chip, at most 9 characters. */
static const char *SmallUi_StateName(uint8_t status)
{
  static const char *const names[] = {
    "BOOTING", "PRECHECK", "NEED PROF", "DISABLED", "ENABLING", "IDLE HOLD", "RT JOINT", "RT CART",
    "NRT JOINT", "NRT CART", "STOPPING", "FAULT", "RECOVERY", "E-STOP", "UNKNOWN", "GRAV FREE",
    "GRAV LOCK", "JOINT IMP", "CART IMP", "JOINT ADM", "CART ADM"};
  return status < sizeof(names) / sizeof(names[0]) ? names[status] : "UNKNOWN";
}

/* One health word, at most 7 characters: OK or the most severe problem. */
enum { SH_OK, SH_FEEDBACK, SH_MISSED, SH_CAN_ERR, SH_BUS_OFF, SH_PC_LOST, SH_NO_ARM, SH_DRIVE };
static uint8_t SmallUi_Health(const AppTasksStatusSnapshot *s, uint8_t stop_reason)
{
  uint8_t responding = 0U;
  if (AppStartup_ArmWaiting(&responding)) return SH_NO_ARM;
  if (s->can_bus_off) return SH_BUS_OFF;
  for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i)
    if (dm_motors[i].feedback_count && dm_motors[i].state >= 8U) return SH_DRIVE;
  if (stop_reason == INFRABOT_STOP_STREAM_LOST || stop_reason == INFRABOT_STOP_MOVE_LOST) return SH_PC_LOST;
  if (s->control_missed) return SH_MISSED;
  if (s->can_tx_errors || s->can_error_passive) return SH_CAN_ERR;
  for (unsigned i = 0U; i < DM_ARM_MOTOR_COUNT; ++i)
    if (dm_motor_runtime.enabled[i] && dm_motors[i].feedback_count == 0U) return SH_FEEDBACK;
  return SH_OK;
}
static const char *SmallUi_HealthName(uint8_t kind)
{
  static const char *const names[] = {"OK", "FB LOST", "MISSED", "CAN ERR", "BUS-OFF", "PC LOST", "NO ARM", "DRV ERR"};
  return kind < sizeof(names) / sizeof(names[0]) ? names[kind] : "?";
}

static void SmallUi_DrawTopStatic(void)
{
  AppLcd_FillRect(0U, 0U, 480U, S_TOP_H, S_BAR);
  AppLcd_FillRect(0U, S_TOP_H, 480U, 1U, S_LINE);
  if (small_ui.page == SP_MENU)
    SmallUi_Label(8U, 9U, small_titles[SP_MENU], S_ACCENT, S_BAR);
  else
  {
    SmallUi_PutButton(S_BACK_X, S_BAR_BTN_Y, S_BACK_W, S_BAR_BTN_H, SA_BACK, 0U, "<", S_BUTTON, S_TEXT);
    SmallUi_Label(S_TITLE_X, 9U, SmallUi_Tr(small_titles[small_ui.page]), S_TEXT, S_BAR);
  }
  SmallUi_PutButton(S_STOP_X, S_BAR_BTN_Y, S_STOP_W, S_BAR_BTN_H, SA_STOP, 0U, TR("STOP"), S_STOP, S_TEXT);
}

/* State chip and health word, right-aligned before STOP; redrawn on change.
 * The health dot is also the 1 Hz heartbeat. */
static void SmallUi_DrawTopLive(const AppTasksStatusSnapshot *s, uint8_t stop_reason, uint8_t heartbeat)
{
  const uint8_t status = RobotController_OperationalStatus();
  const uint8_t health = SmallUi_Health(s, stop_reason);
  const char *state = SmallUi_Tr(SmallUi_StateName(status));
  const char *word = SmallUi_Tr(SmallUi_HealthName(health));
  const uint16_t state_color = AppLcdUi_StatusColor(status);
  const uint16_t health_color = health == SH_OK ? S_GREEN : S_RED;
  const uint16_t word_x = (uint16_t)(S_STOP_X - 8U - SmallUi_Width(word));
  const uint16_t dot_x = (uint16_t)(word_x - 12U);
  UiSlot *const slot = &ui_slots[SS_STATE];
  if (!slot->valid || strcmp(slot->text, state) != 0 || slot->fg != state_color || small_ui.health_drawn != health)
  {
    /* From just after the title (Chinese titles are shorter) to STOP. */
    const uint16_t area_x = small_ui.page == SP_MENU ? 80U :
      (uint16_t)(S_TITLE_X + SmallUi_Width(SmallUi_Tr(small_titles[small_ui.page])) + 4U);
    AppLcd_FillRect(area_x, 4U, (uint16_t)(S_STOP_X - 4U - area_x), 26U, S_BAR);
    SmallUi_Label(word_x, 9U, word, health_color, S_BAR);
    const uint16_t chip_w = (uint16_t)(SmallUi_Width(state) + 16U);
    const uint16_t chip_x = (uint16_t)(dot_x - 8U - chip_w);
    SmallUi_RoundRect(chip_x, 6U, chip_w, 22U, 11U, state_color);
    SmallUi_Center(chip_x, 9U, chip_w, state, S_BLACK, state_color);
    /* The slot only remembers what the chip shows (the chip is not slot text). */
    snprintf(slot->text, sizeof(slot->text), "%s", state);
    slot->length = (uint8_t)AppLcdZh_Columns(state);
    slot->fg = state_color;
    slot->x = (uint16_t)(chip_x + 8U);
    slot->y = 9U;
    slot->size = 16U;
    slot->valid = 1U;
    small_ui.health_drawn = health;
    small_ui.heartbeat_drawn = UINT8_MAX;
  }
  if (small_ui.heartbeat_drawn != heartbeat)
  {
    SmallUi_RoundRect(dot_x, 13U, 8U, 8U, 4U, heartbeat ? health_color : S_LINE);
    small_ui.heartbeat_drawn = heartbeat;
  }
}

/* ---- View switch ------------------------------------------------------------------- */
static void SmallUi_SegmentArea(uint16_t *x, uint16_t *w)
{
  switch (small_ui.page)
  {
    case SP_3D:      *x = 324U; *w = 152U; break;
    case SP_GRAVITY: *x = 236U; *w = 240U; break;
    case SP_NETWORK: *x = 4U;   *w = 236U; break;
    case SP_SETTINGS:    *x = 4U;   *w = 472U; break;
    default:         *x = 188U; *w = 288U; break;
  }
}

static void SmallUi_DrawSegments(void)
{
  const unsigned n = small_view_count[small_ui.page];
  if (n < 2U) return;
  uint16_t x0, width;
  SmallUi_SegmentArea(&x0, &width);
  SmallUi_RoundRect(x0, S_ROW1_Y, width, S_ROW1_H, 6U, S_BAR);
  for (unsigned i = 0U; i < n; ++i)
  {
    const uint16_t x = (uint16_t)(x0 + i * width / n);
    const uint16_t w = (uint16_t)(x0 + (i + 1U) * width / n - x);
    const uint8_t active = i == small_ui.view;
    if (active) SmallUi_RoundRect((uint16_t)(x + 2U), (uint16_t)(S_ROW1_Y + 2U), (uint16_t)(w - 4U),
                                  (uint16_t)(S_ROW1_H - 4U), 6U, S_BUTTON);
    SmallUi_Center(x, (uint16_t)(S_ROW1_Y + 6U), w, SmallUi_Tr(small_view_names[small_ui.page][i]),
                   active ? S_TEXT : S_DIM, active ? S_BUTTON : S_BAR);
    SmallUi_PutButton(x, S_ROW1_Y, w, S_ROW1_H, SA_VIEW, (uint8_t)i, NULL, 0U, 0U);
  }
}

/* Mode button confirmed by the controller: 0 idle, 1 waiting, 2 ramp, 3 active. */
/* Idle it is blue (tap to start), waiting/ramp amber, active green. */
static void SmallUi_ModeButton(uint16_t x, uint16_t w, uint8_t action, const char *name, uint8_t indicator)
{
  char label[24];
  const char *state = indicator == 3U ? TR("ACTIVE") : indicator == 2U ? TR("RAMP") : indicator == 1U ? TR("WAITING") : NULL;
  if (name) name = SmallUi_Tr(name);
  if (name && state) snprintf(label, sizeof(label), TR("%s: %s"), name, state);
  else snprintf(label, sizeof(label), "%s", state ? state : name ? name : TR("ENTER"));
  const uint16_t bg = indicator == 3U ? S_GO : indicator ? S_WAIT : S_START;
  SmallUi_PutButton(x, S_ROW1_Y, w, S_ROW1_H, action, 0U, label, bg, S_TEXT);
}

/* ---- Parameter list with one slider --------------------------------------------------- */
enum { SK_GRAVITY, SK_IMP, SK_ADM };
typedef struct
{
  const char *name, *unit, *tag;
  float value, applied, lo, hi, step;
  uint8_t decimals;
} SmallParam;
typedef struct
{
  SmallParam p[5];
  uint8_t index[5];
  uint8_t n, kind, bias;
} SmallList;

static const char *const small_adm_names[APP_ADM_PARAMETERS] = {
  "Joint K", "Joint mass", "Linear K", "Rotation K", "Virtual mass", "Damping zeta", "Static ratio",
  "Friction beta", "Drive gain", "Elbow null K", "Elbow null D", "Return speed"};

static void SmallUi_ListImpedance(SmallList *l, const uint8_t *index, uint8_t n)
{
  AppImpedanceParameters params;
  AppImpedance_GetParameters(&params);
  l->kind = SK_IMP;
  l->n = n;
  for (uint8_t i = 0U; i < n; ++i)
  {
    const uint8_t k = index[i];
    l->index[i] = k;
    l->p[i] = (SmallParam){imp_names[k], imp_units[k], NULL, params.requested[k], params.applied[k],
                           AppImpedance_ParameterMin(k), AppImpedance_ParameterMax(k), imp_steps[k], 2U};
  }
}

static void SmallUi_ListAdmittance(SmallList *l, const uint8_t *index, uint8_t n)
{
  AppAdmittanceParameters params;
  AppAdmittance_GetParameters(&params);
  l->kind = SK_ADM;
  l->n = n;
  for (uint8_t i = 0U; i < n; ++i)
  {
    const uint8_t k = index[i];
    /* Used by both admittance modes: the same RAM value on both pages. */
    const uint8_t shared = k == APP_ADM_ZETA || k == APP_ADM_STATIC_RATIO || k == APP_ADM_FRICTION ||
                           k == APP_ADM_DRIVE_GAIN;
    l->index[i] = k;
    l->p[i] = (SmallParam){small_adm_names[k], adm_units[k], shared ? "shared" : NULL, params.requested[k],
                           params.applied[k], AppAdmittance_ParameterMin(k), AppAdmittance_ParameterMax(k),
                           adm_steps[k], (k == APP_ADM_LINEAR_K || k == APP_ADM_RETURN_SPEED) ? 1U : 2U};
  }
}

/* The parameter list of the current page and view; 0 when it shows none. */
static uint8_t SmallUi_PageList(SmallList *l)
{
  static const uint8_t joint_imp[2] = {APP_IMP_JOINT_K, APP_IMP_JOINT_D};
  static const uint8_t cart_imp[5] = {APP_IMP_TRANSLATION_K, APP_IMP_TRANSLATION_D, APP_IMP_ROTATION_K,
                                      APP_IMP_ROTATION_D, APP_IMP_JOINT_DAMPING};
  static const uint8_t joint_adm[3] = {APP_ADM_JOINT_K, APP_ADM_JOINT_MASS, APP_ADM_ZETA};
  static const uint8_t cart_adm[5] = {APP_ADM_LINEAR_K, APP_ADM_ROTATION_K, APP_ADM_MASS, APP_ADM_ZETA,
                                      APP_ADM_RETURN_SPEED};
  static const uint8_t shared_adm[3] = {APP_ADM_STATIC_RATIO, APP_ADM_FRICTION, APP_ADM_DRIVE_GAIN};
  static const uint8_t elbow_adm[2] = {APP_ADM_NULL_K, APP_ADM_NULL_D};
  memset(l, 0, sizeof(*l));
  switch (small_ui.page)
  {
    case SP_GRAVITY:
    {
      if (small_ui.view != 0U) return 0U;
      AppGravityParameters p;
      AppGravity_GetParameters(&p);
      l->kind = SK_GRAVITY;
      l->n = 2U;
      l->index[0] = APP_GRAVITY_DAMPING;
      l->index[1] = APP_GRAVITY_STIFFNESS;
      l->p[0] = (SmallParam){"Damping Kd", "Nm s/rad", NULL, p.value[APP_GRAVITY_DAMPING],
                             p.applied[APP_GRAVITY_DAMPING], AppGravity_ParameterMin(APP_GRAVITY_DAMPING),
                             AppGravity_ParameterMax(APP_GRAVITY_DAMPING), .05f, 2U};
      l->p[1] = (SmallParam){"Lock Kp", "Nm/rad", NULL, p.value[APP_GRAVITY_STIFFNESS],
                             p.applied[APP_GRAVITY_STIFFNESS], AppGravity_ParameterMin(APP_GRAVITY_STIFFNESS),
                             AppGravity_ParameterMax(APP_GRAVITY_STIFFNESS), .5f, 2U};
      return 1U;
    }
    case SP_JOINT_IMP: SmallUi_ListImpedance(l, joint_imp, 2U); return 1U;
    case SP_CART_IMP:
      if (small_ui.view != 0U) return 0U;
      SmallUi_ListImpedance(l, cart_imp, 5U);
      return 1U;
    case SP_JOINT_ADM:
      if (small_ui.view == 0U) SmallUi_ListAdmittance(l, joint_adm, 3U);
      else if (small_ui.view == 1U) { SmallUi_ListAdmittance(l, shared_adm, 3U); l->bias = 1U; }
      else return 0U;
      return 1U;
    case SP_CART_ADM:
      if (small_ui.view == 0U) SmallUi_ListAdmittance(l, cart_adm, 5U);
      else if (small_ui.view == 1U) { SmallUi_ListAdmittance(l, shared_adm, 3U); l->bias = 1U; }
      else if (small_ui.view == 2U) SmallUi_ListAdmittance(l, elbow_adm, 2U);
      else return 0U;
      return 1U;
    default:
      return 0U;
  }
}

static AppLcdSlider SmallUi_ListSlider(float step)
{
  return (AppLcdSlider){S_LIST_X + 12U, S_BODY_Y + S_BODY_H - 40U, S_LIST_W - 24U, 40U, step};
}

static void SmallUi_ListSet(const SmallList *l, float value)
{
  const uint8_t k = l->index[small_ui.param];
  if (l->kind == SK_GRAVITY) (void)AppGravity_SetParameter(k, value);
  else if (l->kind == SK_IMP) (void)AppImpedance_SetParameter(k, value);
  else (void)AppAdmittance_SetParameter(k, value);
}

static void SmallUi_DrawList(const SmallList *l, uint8_t enter)
{
  const uint16_t x = S_LIST_X, y = S_BODY_Y;
  if (small_ui.param >= l->n) small_ui.param = 0U;
  if (enter) SmallUi_Card(x, y, S_LIST_W, S_BODY_H, NULL);
  const uint8_t moved = small_ui.param_drawn != small_ui.param;
  for (unsigned i = 0U; i < l->n; ++i)
  {
    const SmallParam *p = &l->p[i];
    const uint16_t ry = (uint16_t)(y + 6U + i * S_LIST_ROW_H);
    const uint8_t selected = i == small_ui.param;
    const uint16_t bg = selected ? S_BUTTON : S_CARD;
    if (enter || (moved && (selected || i == small_ui.param_drawn)))
    {
      AppLcd_FillRect((uint16_t)(x + 4U), (uint16_t)(ry - 3U), (uint16_t)(S_LIST_W - 8U), S_LIST_ROW_H, S_CARD);
      if (selected) SmallUi_RoundRect((uint16_t)(x + 4U), (uint16_t)(ry - 3U), (uint16_t)(S_LIST_W - 8U), S_LIST_ROW_H, 4U, bg);
      ui_slots[SS_ROW(20U + i)].valid = 0U;
    }
    char value[16], right[32], row[96];
    AppLcdUi_Fixed(value, sizeof(value), p->value, p->decimals);
    if (p->unit[0]) snprintf(right, sizeof(right), "%s %s", value, p->unit);
    else snprintf(right, sizeof(right), "%s", value);
    /* Name on the left, value and unit on the right. */
    const char *name = SmallUi_Tr(p->name);
    const unsigned used = AppLcdZh_Columns(name) + AppLcdZh_Columns(right);
    snprintf(row, sizeof(row), "%s%*s%s", name, (int)(used < S_LIST_COLS ? S_LIST_COLS - used : 1U), "", right);
    SmallUi_Text(SS_ROW(20U + i), (uint16_t)(x + 10U), ry, S_LIST_COLS, row, selected ? S_TEXT : S_DIM, bg);
  }
  /* The selected parameter's applied value (ramps to the request) and range. */
  const SmallParam *sel = &l->p[small_ui.param];
  char applied[16], lo[16], hi[16];
  AppLcdUi_Fixed(applied, sizeof(applied), sel->applied, sel->decimals);
  AppLcdUi_Fixed(lo, sizeof(lo), sel->lo, sel->decimals);
  AppLcdUi_Fixed(hi, sizeof(hi), sel->hi, sel->decimals);
  SmallUi_Textf(SS_ROW(25U), (uint16_t)(x + 10U), (uint16_t)(y + S_BODY_H - 58U), S_LIST_COLS, S_DIM, S_CARD,
                TR("applied %s [%s..%s]%s%s"), applied, lo, hi, sel->tag ? " " : "", sel->tag ? SmallUi_Tr(sel->tag) : "");
  const AppLcdSlider slider = SmallUi_ListSlider(sel->step);
  const uint16_t knob = AppLcdSlider_Position(&slider, sel->value, sel->lo, sel->hi);
  if (enter || moved || knob != small_ui.knob_drawn)
  {
    AppLcd_FillRect((uint16_t)(slider.x - 7U), (uint16_t)(slider.y + 11U), (uint16_t)(slider.width + 14U), 18U, S_CARD);
    AppLcd_FillRect(slider.x, (uint16_t)(slider.y + 17U), slider.width, 6U, S_LINE);
    AppLcd_FillRect(slider.x, (uint16_t)(slider.y + 17U), knob, 6U, S_ACCENT);
    SmallUi_RoundRect((uint16_t)(slider.x + knob - 7U), (uint16_t)(slider.y + 11U), 14U, 18U, 4U, S_ACCENT);
    small_ui.knob_drawn = knob;
  }
  small_ui.param_drawn = small_ui.param;
}

/* Finger down on the list card (not a button): the slider captures the
 * gesture until release; a row selects that parameter. */
static void SmallUi_ListPress(const SmallList *l)
{
  const AppLcdSlider slider = SmallUi_ListSlider(l->p[small_ui.param < l->n ? small_ui.param : 0U].step);
  AppLcdSlider_Capture(&ui_slider, &slider, 1U, ui_touch.x, ui_touch.y);
  if (ui_slider.index == 0U)
  {
    const SmallParam *p = &l->p[small_ui.param];
    SmallUi_ListSet(l, AppLcdSlider_Value(&slider, ui_touch.x, p->lo, p->hi));
    return;
  }
  if (ui_touch.x >= S_LIST_X && ui_touch.x < S_LIST_X + S_LIST_W && ui_touch.y >= S_BODY_Y + 3U)
  {
    const unsigned row = (unsigned)(ui_touch.y - (S_BODY_Y + 3U)) / S_LIST_ROW_H;
    if (row < l->n) small_ui.param = (uint8_t)row;
  }
}

static void SmallUi_ListDrag(void)
{
  SmallList l;
  if (!SmallUi_PageList(&l) || small_ui.param >= l.n) { ui_slider.index = APP_LCD_SLIDER_NONE; return; }
  const SmallParam *p = &l.p[small_ui.param];
  const AppLcdSlider slider = SmallUi_ListSlider(p->step);
  SmallUi_ListSet(&l, AppLcdSlider_Value(&slider, ui_touch.x, p->lo, p->hi));
}

/* Joint torque card: J1..J7, first column bright, second dim. */
static void SmallUi_DrawTorques(const char *title, const float a[7], const float b[7], uint8_t enter)
{
  if (enter) SmallUi_Card(S_SIDE_X, S_BODY_Y, S_SIDE_W, S_BODY_H, SmallUi_Tr(title));
  for (unsigned i = 0U; i < 7U; ++i)
  {
    char va[16], vb[16];
    AppLcdUi_Fixed(va, sizeof(va), a[i], 2U);
    AppLcdUi_Fixed(vb, sizeof(vb), b[i], 2U);
    const uint16_t y = (uint16_t)(S_BODY_Y + 24U + i * 20U);
    SmallUi_Textf(SS_SIDE(i), S_SIDE_X + 8U, y, 10U, S_TEXT, S_CARD, "J%u %7s", i + 1U, va);
    SmallUi_Textf(SS_SIDE(7U + i), S_SIDE_X + 96U, y, 8U, S_DIM, S_CARD, "%7s", vb);
  }
}

/* ---- Menu --------------------------------------------------------------------------- */
enum { ST_IDLE, ST_LIVE, ST_WARN };
static uint8_t SmallUi_TileSubtitle(uint8_t page, const AppTasksStatusSnapshot *s, const AppGravityStatus *g,
                                    const InfrabotSafetyStatus *stop, char *out, size_t size)
{
  switch (page)
  {
    case SP_ARM:
    {
      uint8_t responding = 0U;
      if (AppStartup_ArmWaiting(&responding)) { snprintf(out, size, TR("Waiting for arm power %u/7"), responding); return ST_WARN; }
      if (stop->stop_reason)
      { snprintf(out, size, TR("STOP: %s"), SmallUi_Tr(AppLcdUi_StopReasonName(stop->stop_reason))); return ST_WARN; }
      unsigned on = 0U;
      for (unsigned i = 0U; i < DM_ARM_MOTOR_COUNT; ++i) on += dm_motor_runtime.enabled[i] != 0U;
      if (on) snprintf(out, size, TR("Enabled %u/7"), on);
      else snprintf(out, size, "%s", TR("Disabled"));
      return ST_IDLE;
    }
    case SP_3D:
    {
      char xyz[3][16], rpy[3][16];
      SmallUi_Pose(xyz, rpy);
      snprintf(out, size, "TCP %s %s %s", xyz[0], xyz[1], xyz[2]);
      return ST_IDLE;
    }
    case SP_GRAVITY:
      if (!g->recovering && g->mode == DM_ETH_CONTROL_GRAVITY_FREE) { snprintf(out, size, "%s", TR("FREE active")); return ST_LIVE; }
      if (!g->recovering && g->mode == DM_ETH_CONTROL_GRAVITY_LOCK) { snprintf(out, size, "%s", TR("LOCK active")); return ST_LIVE; }
      snprintf(out, size, "%s", TR("Hand-guide or lock pose"));
      return ST_IDLE;
    case SP_NETWORK:
    {
      AppEthernetInfo info;
      AppEthernet_GetInfo(&info);
      char ip[20];
      AppLcdUi_FormatIp(ip, sizeof(ip), info.address.ip);
      snprintf(out, size, TR("%s  link %s"), ip, info.link_up ? TR("UP") : TR("DOWN"));
      return info.link_up ? ST_IDLE : ST_WARN;
    }
    case SP_JOINT_IMP: case SP_CART_IMP: case SP_JOINT_ADM: case SP_CART_ADM:
    {
      static const uint8_t modes[4] = {DM_ETH_CONTROL_JOINT_IMPEDANCE, DM_ETH_CONTROL_CARTESIAN_IMPEDANCE,
                                       DM_ETH_CONTROL_JOINT_ADMITTANCE, DM_ETH_CONTROL_CARTESIAN_ADMITTANCE};
      static const char *const idle[4] = {"Spring on J1-J7", "Spring at the TCP", "Virtual mass on J1-J7",
                                          "Virtual mass at the TCP"};
      const unsigned k = (unsigned)(page - SP_JOINT_IMP);
      if (!g->recovering && g->mode == modes[k])
      {
        if (g->entry_permille < 1000U) snprintf(out, size, TR("Entering %u%%"), g->entry_permille / 10U);
        else snprintf(out, size, "%s", TR("ACTIVE"));
        return ST_LIVE;
      }
      snprintf(out, size, "%s", SmallUi_Tr(idle[k]));
      return ST_IDLE;
    }
    case SP_DIAG:
    {
      const uint8_t can_bad = s->can_bus_off || s->can_tx_errors || s->can_error_passive;
      snprintf(out, size, TR("miss %lu  CAN %s"), (unsigned long)s->control_missed,
               s->can_bus_off ? TR("BUS-OFF") : can_bad ? TR("errors") : TR("ok"));
      return (s->control_missed || can_bad) ? ST_WARN : ST_IDLE;
    }
    default:
      snprintf(out, size, TR("FW %s"), APP_FIRMWARE_VERSION);
      return ST_IDLE;
  }
}

static void SmallUi_DrawMenu(const AppTasksStatusSnapshot *s, const InfrabotSafetyStatus *stop, uint8_t enter)
{
  AppGravityStatus g;
  AppGravity_Status(&g);
  for (unsigned i = 0U; i < 10U; ++i)
  {
    char subtitle[64];
    const uint8_t state = SmallUi_TileSubtitle(small_menu_pages[i], s, &g, stop, subtitle, sizeof(subtitle));
    const uint16_t bg = state == ST_LIVE ? S_LIVE_BG : state == ST_WARN ? S_WARN_BG : S_CARD;
    const uint16_t mark = state == ST_LIVE ? S_GREEN : S_ORANGE;
    const uint16_t x = S_TILE_X(i), y = S_TILE_Y(i);
    if (enter || small_ui.tile_drawn[i] != state)
    {
      SmallUi_RoundRect(x, y, S_TILE_W, S_TILE_H, 8U, bg);
      if (state != ST_IDLE) AppLcd_FillRect(x, (uint16_t)(y + 8U), 3U, (uint16_t)(S_TILE_H - 16U), mark);
      SmallUi_Label((uint16_t)(x + 12U), (uint16_t)(y + 5U), SmallUi_Tr(small_titles[small_menu_pages[i]]), S_TEXT, bg);
      ui_slots[SS_ROW(i)].valid = 0U;
      small_ui.tile_drawn[i] = state;
    }
    SmallUi_Text(SS_ROW(i), (uint16_t)(x + 12U), (uint16_t)(y + 22U), 26U, subtitle, state == ST_IDLE ? S_DIM : mark, bg);
    if (enter) SmallUi_PutButton(x, y, S_TILE_W, S_TILE_H, SA_TILE, small_menu_pages[i], NULL, 0U, 0U);
  }
}

/* ---- Arm & Joints ------------------------------------------------------------------- */
static void SmallUi_DrawArm(const InfrabotSafetyStatus *stop, uint8_t enter)
{
  if (enter)
  {
    SmallUi_PutButton(4U, S_ROW1_Y, 154U, S_ROW1_H, SA_ENABLE, 0U, TR("ENABLE ARM"), S_GO, S_TEXT);
    SmallUi_PutButton(163U, S_ROW1_Y, 154U, S_ROW1_H, SA_DISABLE, 0U, TR("DISABLE ARM"), S_BUTTON, S_TEXT);
    SmallUi_PutButton(322U, S_ROW1_Y, 154U, S_ROW1_H, SA_RESUME, 0U, TR("RESUME"), S_GO, S_TEXT);
    SmallUi_Card(4U, S_BODY_Y, 368U, S_BODY_H, NULL);
    char header[96], q[24], cmd[24], tq[24], t[24];
    snprintf(header, sizeof(header), "J %s%s%s%s %s", SmallUi_Pad(q, sizeof(q), TR("q deg"), 8U, 1U),
             SmallUi_Pad(cmd, sizeof(cmd), TR("cmd"), 8U, 1U), SmallUi_Pad(tq, sizeof(tq), TR("tq Nm"), 7U, 1U),
             SmallUi_Pad(t, sizeof(t), TR("T C"), 4U, 1U), TR("STATE"));
    SmallUi_Label(12U, 98U, header, S_DIM, S_CARD);
    AppLcd_FillRect(12U, 115U, 352U, 1U, S_LINE);
    SmallUi_Card(376U, S_BODY_Y, 100U, S_BODY_H, "TCP mm");
    SmallUi_Label(384U, 178U, "RPY deg", S_DIM, S_CARD);
  }
  AppLocalControlStatus local;
  AppLocalControl_Status(&local);
  uint8_t responding = 0U;
  char line[96];
  uint16_t color = S_DIM;
  if (AppStartup_ArmWaiting(&responding))
  { snprintf(line, sizeof(line), TR("Waiting for arm power: %u/7 motors respond"), responding); color = S_ORANGE; }
  else if (local.result) { snprintf(line, sizeof(line), "%s", SmallUi_Tr(AppLocalControl_Message(&local))); color = S_ORANGE; }
  else if (stop->stop_reason)
  {
    const char *phase = stop->stop_phase == INFRABOT_STOP_PHASE_BRAKING ? TR("braking") :
                        stop->stop_phase == INFRABOT_STOP_PHASE_HOLDING ? TR("holding, no PC (stays enabled)") :
                        TR("stopped, holding");
    snprintf(line, sizeof(line), TR("STOP %s: %s"), SmallUi_Tr(AppLcdUi_StopReasonName(stop->stop_reason)), phase);
    color = S_ORANGE;
  }
  else snprintf(line, sizeof(line), "%s", SmallUi_Tr(AppLocalControl_Message(&local)));
  SmallUi_Status(line, color);

  const uint8_t limit = AppSafety_TorqueLimitMask();
  for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i)
  {
    const DM_Motor_t *m = &dm_motors[i];
    const float d = DM_MOTOR_DIRECTION(i);
    char q[16], cmd[16], tq[16];
    AppLcdUi_Fixed(q, sizeof(q), (m->position - dm_motor_runtime.zero_offset[i]) * d * 57.2957795f, 2U);
    if (dm_motor_runtime.control_enabled && dm_motor_runtime.enabled[i])
      AppLcdUi_Fixed(cmd, sizeof(cmd), (dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) * d * 57.2957795f, 2U);
    else snprintf(cmd, sizeof(cmd), "-");
    AppLcdUi_Fixed(tq, sizeof(tq), m->torque * d, 2U);
    const char *state = !m->feedback_count ? TR("NO FEEDBACK") :
                        m->state <= 1U ? (dm_motor_runtime.enabled[i] ? TR("OK") : TR("OFF")) :
                        SmallUi_Tr(AppTasks_MotorStateName(m->state));
    const uint16_t c = m->state >= 8U ? S_RED : (i < DM_ARM_MOTOR_COUNT && (limit & (1U << i))) ? S_ORANGE :
                       m->feedback_count ? S_TEXT : S_DIM;
    const uint8_t slot = SS_ROW(i);
    const uint8_t full = !ui_slots[slot].valid || ui_slots[slot].fg != c;
    const uint16_t y = (uint16_t)(118U + i * 18U);
    SmallUi_Textf(slot, 12U, y, 44U, c, S_CARD, "J%u%8s%8s%7s%4u %s", i + 1U, q, cmd, tq,
                  (unsigned)m->temperature_mos, state);
    /* Enable dot in the blank third column (q fits in 7 of its 8). */
    const uint8_t on = dm_motor_runtime.enabled[i] != 0U;
    if (full || small_ui.dot_drawn[i] != on)
    {
      SmallUi_RoundRect(29U, (uint16_t)(y + 4U), 8U, 8U, 4U, on ? S_GREEN : S_LINE);
      small_ui.dot_drawn[i] = on;
    }
  }
  char xyz[3][16], rpy[3][16];
  SmallUi_Pose(xyz, rpy);
  static const char axis[6] = {'X', 'Y', 'Z', 'R', 'P', 'Y'};
  for (unsigned i = 0U; i < 6U; ++i)
    SmallUi_Textf(SS_SIDE(i), 384U, (uint16_t)(i < 3U ? 118U + i * 18U : 198U + (i - 3U) * 18U), 10U, S_TEXT,
                  S_CARD, "%c %8s", axis[i], i < 3U ? xyz[i] : rpy[i - 3U]);
}

/* ---- Gravity compensation ------------------------------------------------------------- */
static uint8_t SmallUi_GravityIndicator(const AppGravityStatus *g, const AppLocalControlStatus *c, uint8_t lock)
{
  const uint8_t mode = lock ? DM_ETH_CONTROL_GRAVITY_LOCK : DM_ETH_CONTROL_GRAVITY_FREE;
  if (!g->recovering && g->mode == mode) return g->entry_permille >= 1000U ? 3U : 2U;
  return (c->pending_actions & (1U << (lock ? APP_LOCAL_LOCK : APP_LOCAL_FREE))) ? 1U : 0U;
}

static void SmallUi_DrawGravity(uint8_t enter)
{
  AppGravityStatus g;
  AppLocalControlStatus local;
  AppGravity_Status(&g);
  AppLocalControl_Status(&local);
  for (uint8_t lock = 0U; lock < 2U; ++lock)
  {
    const uint8_t indicator = SmallUi_GravityIndicator(&g, &local, lock);
    if (enter || indicator != small_ui.indicator_drawn[lock])
    {
      SmallUi_ModeButton(lock ? 120U : 4U, 112U, lock ? SA_LOCK : SA_FREE, lock ? "LOCK" : "FREE", indicator);
      small_ui.indicator_drawn[lock] = indicator;
    }
  }
  char line[96];
  uint16_t color = S_DIM;
  const uint8_t active = g.mode == DM_ETH_CONTROL_GRAVITY_FREE || g.mode == DM_ETH_CONTROL_GRAVITY_LOCK;
  if (local.result && (local.action == APP_LOCAL_FREE || local.action == APP_LOCAL_LOCK))
  { snprintf(line, sizeof(line), TR("Rejected: %s"), SmallUi_Tr(AppLocalControl_Message(&local))); color = S_ORANGE; }
  else if (g.recovering) { snprintf(line, sizeof(line), "%s", SmallUi_Tr(ComplianceState(&g))); color = S_ORANGE; }
  else if (active) { snprintf(line, sizeof(line), "%s", SmallUi_Tr(ComplianceState(&g))); color = S_TEXT; }
  else if (g.fault.sequence)
  {
    snprintf(line, sizeof(line), TR("Last stop J%u: %s"), g.fault.joint + 1U, SmallUi_Tr(AppGravity_CheckName(g.fault.check)));
    color = S_ORANGE;
  }
  else snprintf(line, sizeof(line), "%s", TR("Enable the arm on Arm & Joints, then FREE or LOCK"));
  SmallUi_Status(line, color);

  if (small_ui.view == 0U)
  {
    SmallList list;
    (void)SmallUi_PageList(&list);
    SmallUi_DrawList(&list, enter);
    char joints[28] = "";
    unsigned used = 0U;
    for (unsigned i = 0U; i < 7U; ++i)
      if (g.speed_limited_mask & (1U << i)) used += (unsigned)snprintf(joints + used, sizeof(joints) - used, "J%u ", i + 1U);
    SmallUi_Textf(SS_ROW(26U), 14U, (uint16_t)(S_BODY_Y + 54U), S_LIST_COLS, S_DIM, S_CARD, TR("Model %s"),
                  ArmGravity_ModelName());
    SmallUi_Textf(SS_ROW(27U), 14U, (uint16_t)(S_BODY_Y + 72U), S_LIST_COLS, g.speed_limited_mask ? S_ORANGE : S_DIM,
                  S_CARD, TR("Braking: %s"), used ? joints : TR("none"));
    SmallUi_DrawTorques(TR("FF / Drive Nm"), g.torque, g.feedback_torque, enter);
    return;
  }
  /* DETAILS: model, last protection stop, braking joints, timing. */
  if (enter) SmallUi_Card(4U, S_BODY_Y, 472U, S_BODY_H, NULL);
  const char *hash = ArmGravity_ModelHash();
  const uint16_t y0 = (uint16_t)(S_BODY_Y + 6U);
  SmallUi_Textf(SS_ROW(0U), 12U, y0, 57U, S_TEXT, S_CARD, TR("Model %s"), ArmGravity_ModelName());
  SmallUi_Textf(SS_ROW(1U), 12U, (uint16_t)(y0 + 18U), 57U, S_DIM, S_CARD, "SHA256 %.32s", hash);
  SmallUi_Textf(SS_ROW(2U), 12U, (uint16_t)(y0 + 36U), 57U, S_DIM, S_CARD, "       %.32s",
                strlen(hash) > 32U ? hash + 32 : "");
  if (g.fault.sequence)
  {
    char value[20], limit[20], joint[12];
    const uint8_t check = g.fault.check;
    const char *unit = check == APP_GRAVITY_CHECK_SPEED ? "rad/s" :
                       (check == APP_GRAVITY_CHECK_LOWER || check == APP_GRAVITY_CHECK_UPPER) ? "rad" :
                       check == APP_GRAVITY_CHECK_BUDGET ? "us" : check == APP_GRAVITY_CHECK_FEEDBACK ? "ms" :
                       check == APP_GRAVITY_CHECK_TEMPERATURE ? "C" : "";
    AppLcdUi_Fixed(value, sizeof(value), g.fault.value, 3U);
    AppLcdUi_Fixed(limit, sizeof(limit), g.fault.limit, 3U);
    if (g.fault.joint < 7U) snprintf(joint, sizeof(joint), "J%u", g.fault.joint + 1U);
    else snprintf(joint, sizeof(joint), "%s", TR("arm"));
    SmallUi_Textf(SS_ROW(3U), 12U, (uint16_t)(y0 + 60U), 57U, S_ORANGE, S_CARD, TR("Last stop %s: %s"), joint,
                  SmallUi_Tr(AppGravity_CheckName(check)));
    SmallUi_Textf(SS_ROW(4U), 12U, (uint16_t)(y0 + 78U), 57U, S_ORANGE, S_CARD, TR("value %s / limit %s %s"), value,
                  limit, unit);
  }
  else
  {
    SmallUi_Text(SS_ROW(3U), 12U, (uint16_t)(y0 + 60U), 57U, TR("Last stop: none"), S_DIM, S_CARD);
    SmallUi_Text(SS_ROW(4U), 12U, (uint16_t)(y0 + 78U), 57U, "", S_DIM, S_CARD);
  }
  char joints[28] = "";
  unsigned used = 0U;
  for (unsigned i = 0U; i < 7U; ++i)
    if (g.speed_limited_mask & (1U << i)) used += (unsigned)snprintf(joints + used, sizeof(joints) - used, "J%u ", i + 1U);
  SmallUi_Textf(SS_ROW(5U), 12U, (uint16_t)(y0 + 102U), 57U, g.speed_limited_mask ? S_ORANGE : S_DIM, S_CARD,
                TR("Braking (speed/joint limit): %s"), used ? joints : TR("none"));
  SmallUi_Textf(SS_ROW(6U), 12U, (uint16_t)(y0 + 120U), 57U, S_DIM, S_CARD, TR("1000 Hz | calc %lu/%lu us"),
                (unsigned long)g.last_us, (unsigned long)g.max_us);
  SmallUi_Text(SS_ROW(7U), 12U, (uint16_t)(y0 + 138U), 57U, TR("RAM settings, reset on reboot. STOP holds enabled."),
               S_DIM, S_CARD);
}

/* ---- Impedance ---------------------------------------------------------------------------- */
static void SmallUi_ModeStatus(uint8_t indicator, const AppGravityStatus *g, const AppLocalControlStatus *c,
                               uint8_t action, const char *idle, const char *active)
{
  char line[96];
  uint16_t color = S_DIM;
  if (c->result && c->action == action && indicator < 2U)
  { snprintf(line, sizeof(line), TR("Rejected: %s"), SmallUi_Tr(AppLocalControl_Message(c))); color = S_ORANGE; }
  else if (g->recovering) { snprintf(line, sizeof(line), "%s", TR("STOPPING: restoring hold gains")); color = S_ORANGE; }
  else if (indicator == 3U) { snprintf(line, sizeof(line), "%s", active); color = S_TEXT; }
  else if (indicator == 2U) { snprintf(line, sizeof(line), TR("Entry ramp %u%%; please wait"), g->entry_permille / 10U); color = S_YELLOW; }
  else if (indicator == 1U) { snprintf(line, sizeof(line), "%s", TR("Request queued; waiting for the controller")); color = S_YELLOW; }
  else if (g->fault.sequence)
  {
    snprintf(line, sizeof(line), TR("Last stop J%u: %s"), g->fault.joint + 1U, SmallUi_Tr(AppGravity_CheckName(g->fault.check)));
    color = S_ORANGE;
  }
  else snprintf(line, sizeof(line), "%s", idle);
  SmallUi_Status(line, color);
}

static void SmallUi_LiveRow(unsigned row, const char *text, uint16_t color)
{
  SmallUi_Text(SS_ROW(row), 12U, (uint16_t)(S_BODY_Y + 6U + row * 18U), 57U, text, color, S_CARD);
}

static void SmallUi_DrawImpedance(uint8_t cartesian, uint8_t enter)
{
  AppGravityStatus g;
  AppLocalControlStatus c;
  AppGravity_Status(&g);
  AppLocalControl_Status(&c);
  const uint8_t indicator = ImpedanceIndicator(&g, &c, cartesian);
  if (enter || indicator != small_ui.indicator_drawn[0])
  {
    SmallUi_ModeButton(4U, cartesian ? 180U : 472U, SA_ENTER, NULL, indicator);
    small_ui.indicator_drawn[0] = indicator;
  }
  SmallUi_ModeStatus(indicator, &g, &c, cartesian ? APP_LOCAL_CART_IMP : APP_LOCAL_JOINT_IMP,
                     cartesian ? TR("TCP spring + damper; no nullspace posture spring") :
                                 TR("Spring + damper on J1-J7; equilibrium = pose on entry"),
                     cartesian ? TR("Entered; TCP equilibrium latched. STOP holds enabled.") :
                                 TR("Entered; equilibrium latched. STOP holds enabled."));
  SmallList list;
  if (SmallUi_PageList(&list))
  {
    SmallUi_DrawList(&list, enter);
    SmallUi_DrawTorques(TR("FF / Drive Nm"), g.torque, g.feedback_torque, enter);
    return;
  }
  /* LIVE: the latest control sample of CART IMP. */
  if (enter) SmallUi_Card(4U, S_BODY_Y, 472U, S_BODY_H, NULL);
  const ArmImpedanceDiagnostics *d = &g.impedance;
  const uint8_t live = g.mode == DM_ETH_CONTROL_CARTESIAN_IMPEDANCE && d->valid;
  char line[100];
  for (unsigned row = 0U; row < 8U; ++row)
  {
    line[0] = '\0';
    if (!live) { if (row == 0U) snprintf(line, sizeof(line), "%s", TR("No live CART IMP sample; enter CART IMP first.")); }
    else switch (row)
    {
      case 0: ImpedanceVector(line, sizeof(line), TR("Target mm"), d->target, 1000.f); break;
      case 1: ImpedanceVector(line, sizeof(line), TR("Actual mm"), d->measured, 1000.f); break;
      case 2: ImpedanceVector(line, sizeof(line), TR("Error  mm"), d->position_error, 1000.f); break;
      case 3: ImpedanceVector(line, sizeof(line), TR("Rot err deg"), d->rotation_error, 57.2957795f); break;
      case 4: ImpedanceVector(line, sizeof(line), TR("Virtual F N"), d->force, 1.f); break;
      case 5: ImpedanceVector(line, sizeof(line), TR("Virtual M Nm"), d->moment, 1.f); break;
      case 6: snprintf(line, sizeof(line), "cap=%02X slew=%02X errcap=%02X brake=%02X", g.torque_clipped,
                       g.torque_slew_limited, d->error_clamped, g.speed_limited_mask); break;
      default:
      {
        unsigned joint = 0U;
        for (unsigned i = 1U; i < 7U; ++i)
          if (fabsf(g.impedance_torque[i]) > fabsf(g.impedance_torque[joint])) joint = i;
        char value[16];
        AppLcdUi_Fixed(value, sizeof(value), g.impedance_torque[joint], 3U);
        snprintf(line, sizeof(line), TR("Largest correction J%u: %s Nm"), joint + 1U, value);
      }
    }
    SmallUi_LiveRow(row, line, row == 2U ? S_ACCENT : row == 6U ? S_YELLOW : S_TEXT);
  }
  SmallUi_LiveRow(8U, TR("Actual = FK feedback | virtual wrench != contact force"), S_DIM);
}

/* ---- Admittance ---------------------------------------------------------------------------- */
static void SmallUi_DrawAdmittance(uint8_t cartesian, uint8_t enter)
{
  static AppAdmittanceStatus s;
  AppGravityStatus g;
  AppLocalControlStatus c;
  AppAdmittance_Status(&s);
  AppGravity_Status(&g);
  AppLocalControl_Status(&c);
  const uint8_t indicator = AdmittanceIndicator(&g, &c, cartesian);
  if (enter || indicator != small_ui.indicator_drawn[0])
  {
    SmallUi_ModeButton(4U, 180U, SA_ENTER, NULL, indicator);
    small_ui.indicator_drawn[0] = indicator;
  }
  char line[100];
  uint16_t color = S_DIM;
  const uint8_t action = cartesian ? APP_LOCAL_CART_ADM : APP_LOCAL_JOINT_ADM;
  if (c.result && c.action == action && indicator < 2U)
  { snprintf(line, sizeof(line), "%s", TR("Rejected: enable/zero/feedback; 0.03 rad limit margin")); color = S_ORANGE; }
  else if (g.recovering) { snprintf(line, sizeof(line), "%s", TR("STOPPING: restoring hold")); color = S_ORANGE; }
  else if (indicator == 2U) { snprintf(line, sizeof(line), TR("ADM entry ramp %u%%; please wait"), g.entry_permille / 10U); color = S_YELLOW; }
  else if (indicator == 3U && s.drift_frozen) { snprintf(line, sizeof(line), "%s", TR("ADM FROZEN: IK lag")); color = S_ORANGE; }
  else if (indicator == 3U && s.frozen) { snprintf(line, sizeof(line), "%s", TR("ADM FROZEN: overload")); color = S_ORANGE; }
  else if (indicator == 3U && s.returning)
  {
    char value[16];
    const float distance = sqrtf(s.error[0] * s.error[0] + s.error[1] * s.error[1] + s.error[2] * s.error[2]);
    AppLcdUi_Fixed(value, sizeof(value), distance * 1000.f, 1U);
    snprintf(line, sizeof(line), TR("ADM RETURN %s mm"), value);
    color = S_GREEN;
  }
  else if (indicator == 3U) { snprintf(line, sizeof(line), "%s", TR("Drive PD tracks the virtual reference; STOP holds.")); color = S_TEXT; }
  else if (indicator == 1U) { snprintf(line, sizeof(line), "%s", TR("Request queued; waiting for the controller")); color = S_YELLOW; }
  else snprintf(line, sizeof(line), "%s", cartesian ? TR("Virtual mass at the TCP; springs back to the start pose") :
                                                     TR("Virtual mass on J1-J7; springs back to the start pose"));
  SmallUi_Status(line, color);

  SmallList list;
  if (SmallUi_PageList(&list))
  {
    SmallUi_DrawList(&list, enter);
    if (list.bias)
    {
      /* Bias learning: a switch, shared by both admittance modes. */
      const uint8_t on = AppAdmittance_BiasLearning() != 0U;
      const uint16_t y = (uint16_t)(S_BODY_Y + 6U + list.n * S_LIST_ROW_H + 2U);
      if (enter || small_ui.bias_drawn != on)
      {
        SmallUi_PutButton(12U, y, 140U, 22U, SA_BIAS, 0U, on ? TR("Bias learn: ON") : TR("Bias learn: OFF"),
                          on ? S_GO : S_BUTTON, S_TEXT);
        small_ui.bias_drawn = on;
      }
      SmallUi_Text(SS_ROW(27U), 160U, (uint16_t)(y + 3U), 17U, TR("shared"), S_DIM, S_CARD);
    }
    SmallUi_DrawTorques(TR("Ext / PD Nm"), s.external, s.pd, enter);
    return;
  }
  /* LIVE: the latest admittance sample. */
  if (enter) SmallUi_Card(4U, S_BODY_Y, 472U, S_BODY_H, NULL);
  const uint8_t active = indicator >= 2U;
  for (unsigned row = 0U; row < 8U; ++row)
  {
    line[0] = '\0';
    if (!active) { if (row == 0U) snprintf(line, sizeof(line), "%s", TR("No live ADM sample.")); }
    else switch (row)
    {
      case 0: ImpedanceVector(line, sizeof(line), TR("F N"), s.force, 1.f); break;
      case 1: ImpedanceVector(line, sizeof(line), TR("M Nm"), s.moment, 1.f); break;
      case 2: ImpedanceVector(line, sizeof(line), TR("Feff N"), s.effective_force, 1.f); break;
      case 3: ImpedanceVector(line, sizeof(line), TR("e mm"), s.error, 1000.f); break;
      case 4: ImpedanceVector(line, sizeof(line), TR("rot deg"), s.error + 3, 57.29578f); break;
      case 5: ImpedanceVector(line, sizeof(line), TR("vref mm/s"), s.rate, 1000.f); break;
      default:
      {
        const float *v = row == 6U ? s.external : s.tracking;
        unsigned largest = 0U;
        for (unsigned j = 1U; j < 7U; ++j) if (fabsf(v[j]) > fabsf(v[largest])) largest = j;
        char value[16];
        AppLcdUi_Fixed(value, sizeof(value), v[largest] * (row == 6U ? 1.f : 1000.f), row == 6U ? 3U : 1U);
        snprintf(line, sizeof(line), "%s J%u %s %s", row == 6U ? TR("Max ext") : TR("Max track"), largest + 1U, value,
                 row == 6U ? "Nm" : "mrad");
      }
    }
    SmallUi_LiveRow(row, line, S_TEXT);
  }
  snprintf(line, sizeof(line), TR("RAM settings | model calc %lu/%lu us"), (unsigned long)g.last_us, (unsigned long)g.max_us);
  SmallUi_LiveRow(8U, line, S_DIM);
}

/* ---- Network --------------------------------------------------------------------------------- */
/* The address editor's messages (app_lcd_ui.c, English) as shown here. */
static const char *SmallUi_NetMessage(unsigned i, char *out, size_t size)
{
  static const char saved[] = "Saved. Now ";
  const char *m = ui_net.message[i];
  if (strncmp(m, saved, sizeof(saved) - 1U) == 0)
  {
    snprintf(out, size, "%s%s", TR("Saved. Now "), m + sizeof(saved) - 1U);
    return out;
  }
  return SmallUi_Tr(m);
}

/* Also called by AppLcdUi_NetSave before the blocking flash save. */
static void SmallUi_NetMessagesNow(void)
{
  char text[64];
  if (small_ui.page != SP_NETWORK) return;
  const uint8_t edit = small_ui.view == 0U;
  for (unsigned i = 0U; i < 2U; ++i)
    SmallUi_Text(SS_ROW((edit ? 7U : 8U) + i), edit ? 6U : 12U, (uint16_t)(edit ? 234U + i * 18U : 230U + i * 18U),
                 edit ? 58U : 57U, SmallUi_NetMessage(i, text, sizeof(text)), ui_net.message_color, edit ? S_BG : S_CARD);
}

static void SmallUi_DrawNetwork(uint8_t enter)
{
  AppEthernetInfo info;
  AppEthernet_GetInfo(&info);
  if (!ui_net.loaded) AppLcdUi_NetLoadCurrent();
  char ip[20];
  const char *source = info.pending ? TR("APPLYING") : info.from_flash ? TR("SAVED") : TR("DEFAULT");
  if (small_ui.view == 0U)
  {
    if (enter)
    {
      for (uint8_t k = 0U; k < 15U; ++k)
        SmallUi_PutButton((uint16_t)(248U + (k % 3U) * 76U), (uint16_t)(40U + (k / 3U) * 32U), 72U, 28U, SA_KEY, k,
                          SmallUi_Tr(small_key_labels[k]), S_BUTTON, S_TEXT);
      SmallUi_PutButton(248U, 200U, 224U, 28U, SA_KEY, 15U, SmallUi_Tr(small_key_labels[15]), S_GO, S_TEXT);
    }
    AppLcdUi_FormatIp(ip, sizeof(ip), info.address.ip);
    SmallUi_Textf(SS_ROW(0U), 6U, 74U, 29U, S_ACCENT, S_BG, "IP   %s", ip);
    AppLcdUi_FormatIp(ip, sizeof(ip), info.address.netmask);
    SmallUi_Textf(SS_ROW(1U), 6U, 90U, 29U, S_TEXT, S_BG, "MASK %s", ip);
    AppLcdUi_FormatIp(ip, sizeof(ip), info.address.gateway);
    SmallUi_Textf(SS_ROW(2U), 6U, 106U, 29U, S_TEXT, S_BG, "GW   %s", ip);
    SmallUi_Textf(SS_ROW(3U), 6U, 122U, 29U, info.link_up ? S_GREEN : S_ORANGE, S_BG, TR("LINK %s  %s"),
                  info.link_up ? TR("UP") : TR("DOWN"), source);
    for (unsigned i = 0U; i < 3U; ++i)
    {
      const uint8_t selected = ui_net.selected == i;
      SmallUi_Textf(SS_ROW(4U + i), 6U, S_FIELD_Y(i), 29U, selected ? S_YELLOW : S_TEXT, S_BG, TR("%cNEW %-4s %s%s"),
                    selected ? '>' : ' ', i == 0U ? "IP" : i == 1U ? "MASK" : "GW", ui_net.field[i], selected ? "_" : "");
    }
    SmallUi_NetMessagesNow();
    return;
  }
  /* STATUS: the address in use, the PC link and the edit in progress (read only). */
  if (enter) SmallUi_Card(4U, 72U, 472U, 196U, NULL);
  const uint16_t y0 = 78U;
  AppLcdUi_FormatIp(ip, sizeof(ip), info.address.ip);
  SmallUi_Textf(SS_ROW(0U), 12U, y0, 57U, S_ACCENT, S_CARD, "IP      %s  %s", ip, source);
  AppLcdUi_FormatIp(ip, sizeof(ip), info.address.netmask);
  SmallUi_Textf(SS_ROW(1U), 12U, (uint16_t)(y0 + 18U), 57U, S_TEXT, S_CARD, "MASK    %s", ip);
  AppLcdUi_FormatIp(ip, sizeof(ip), info.address.gateway);
  SmallUi_Textf(SS_ROW(2U), 12U, (uint16_t)(y0 + 36U), 57U, S_TEXT, S_CARD, "GATEWAY %s", ip);
  SmallUi_Textf(SS_ROW(3U), 12U, (uint16_t)(y0 + 54U), 57U, S_TEXT, S_CARD, "MAC     %02X:%02X:%02X:%02X:%02X:%02X",
                info.mac[0], info.mac[1], info.mac[2], info.mac[3], info.mac[4], info.mac[5]);
  if (info.client_valid)
  {
    AppLcdUi_FormatIp(ip, sizeof(ip), info.client_ip);
    SmallUi_Textf(SS_ROW(4U), 12U, (uint16_t)(y0 + 72U), 57U, info.link_up ? S_GREEN : S_ORANGE, S_CARD,
                  TR("LINK %s  PC %s:%u"), info.link_up ? TR("UP") : TR("DOWN"), ip, info.client_port);
  }
  else SmallUi_Textf(SS_ROW(4U), 12U, (uint16_t)(y0 + 72U), 57U, info.link_up ? S_GREEN : S_ORANGE, S_CARD,
                     TR("LINK %s  PC none"), info.link_up ? TR("UP") : TR("DOWN"));
  for (unsigned i = 0U; i < 3U; ++i)
    SmallUi_Textf(SS_ROW(5U + i), 12U, (uint16_t)(y0 + 96U + i * 18U), 57U, ui_net.selected == i ? S_YELLOW : S_TEXT,
                  S_CARD, TR("NEW %-7s %s"), ui_field_names[i], ui_net.field[i]);
  SmallUi_NetMessagesNow();
}

/* ---- 3D view ------------------------------------------------------------------------------------ */
static uint8_t SmallUi_Draw3d(uint8_t enter)
{
  uint8_t drawn = 0U;
  if (small_ui.view == 0U)
  {
    if (enter)
    {
      SmallUi_PutButton(324U, 72U, 74U, 26U, SA_VIEW3D, APP_LCD3D_VIEW_HOME, TR("HOME"), S_BUTTON, S_TEXT);
      SmallUi_PutButton(402U, 72U, 74U, 26U, SA_VIEW3D, APP_LCD3D_VIEW_FRONT, TR("FRONT"), S_BUTTON, S_TEXT);
      SmallUi_PutButton(324U, 102U, 74U, 26U, SA_VIEW3D, APP_LCD3D_VIEW_SIDE, TR("SIDE"), S_BUTTON, S_TEXT);
      SmallUi_PutButton(402U, 102U, 74U, 26U, SA_VIEW3D, APP_LCD3D_VIEW_TOP, TR("TOP"), S_BUTTON, S_TEXT);
    }
    const uint32_t start = DWT->CYCCNT;
    drawn = AppLcd3d_Render(0U, S_3D_Y, S_3D_W, S_3D_H);
    if (drawn)
    {
      ui_3d_us = (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);
      if (ui_3d_us > ui_3d_max_us) ui_3d_max_us = ui_3d_us;
    }
  }
  else if (enter) SmallUi_Card(4U, 40U, 312U, 228U, NULL);
  /* Readouts at most 10 Hz, immediately on entry. */
  const TickType_t now = xTaskGetTickCount();
  if (!enter && (TickType_t)(now - ui_3d_text_tick) < pdMS_TO_TICKS(100)) return drawn;
  ui_3d_text_tick = now;
  char xyz[3][16], rpy[3][16];
  SmallUi_Pose(xyz, rpy);
  SmallUi_Text(SS_SIDE(0U), 328U, 136U, 18U, "TCP mm", S_ACCENT, S_BG);
  for (unsigned i = 0U; i < 3U; ++i)
    SmallUi_Textf(SS_SIDE(1U + i), 328U, (uint16_t)(154U + i * 18U), 18U, S_TEXT, S_BG, "%c %9s", "XYZ"[i], xyz[i]);
  if (small_ui.view == 0U)
  {
    SmallUi_Textf(SS_SIDE(4U), 328U, 214U, 18U, S_DIM, S_BG, TR("draw %lu us"), (unsigned long)ui_3d_us);
    SmallUi_Textf(SS_SIDE(5U), 328U, 232U, 18U, S_DIM, S_BG, TR("max  %lu us"), (unsigned long)ui_3d_max_us);
    SmallUi_Text(SS_SIDE(6U), 328U, 250U, 18U, TR("Drag to rotate"), S_DIM, S_BG);
    return drawn;
  }
  SmallUi_Text(SS_SIDE(4U), 328U, 214U, 18U, TR("Model paused while"), S_DIM, S_BG);
  SmallUi_Text(SS_SIDE(5U), 328U, 232U, 18U, TR("RENDER is open"), S_DIM, S_BG);
  /* RENDER: the last frame drawn before this view opened. */
  AppLcd3dTiming t;
  AppLcd3d_Timing(&t);
  const uint32_t div = SystemCoreClock / 1000000U;
  const UiFrameReport *r = &ui_3d_detail_report;
  char line[64];
  for (unsigned row = 0U; row < 12U; ++row)
  {
    switch (row)
    {
      case 0: snprintf(line, sizeof(line), "geo %lu  px %lu us", (unsigned long)(t.prepare_cycles / div), (unsigned long)(t.draw_cycles / div)); break;
      case 1: snprintf(line, sizeof(line), "clr %lu  ras %lu us", (unsigned long)(t.clear_cycles / div), (unsigned long)(t.raster_cycles / div)); break;
      case 2: snprintf(line, sizeof(line), "cln %lu  cpy %lu us", (unsigned long)(t.clean_cycles / div), (unsigned long)(t.blit_cycles / div)); break;
      case 3: snprintf(line, sizeof(line), "area %lu%%  %lu px", (unsigned long)(t.updated_pixels * 100U / (S_3D_W * S_3D_H)), (unsigned long)t.updated_pixels); break;
      case 4: snprintf(line, sizeof(line), "render %lu/%lu us", (unsigned long)ui_3d_us, (unsigned long)ui_3d_max_us); break;
      case 5: snprintf(line, sizeof(line), "UI %lu.%lu  draw %lu.%lu Hz", (unsigned long)(r->ui_hz10 / 10U), (unsigned long)(r->ui_hz10 % 10U),
                       (unsigned long)(r->draw_hz10 / 10U), (unsigned long)(r->draw_hz10 % 10U)); break;
      case 6: snprintf(line, sizeof(line), "frame %lu/%lu us", (unsigned long)r->last_us, (unsigned long)r->max_us); break;
      case 7: snprintf(line, sizeof(line), "gap %lu/%lu us", (unsigned long)r->interval_min_us, (unsigned long)r->interval_max_us); break;
      case 8: snprintf(line, sizeof(line), "over %lu  %s  tiles %lu", (unsigned long)r->over, t.sram_buffer ? "SRAM" : "SDRAM", (unsigned long)t.tiles); break;
      case 9: snprintf(line, sizeof(line), TR("mesh %u triangles"), (unsigned)APP_LCD_MESH_TOTAL_TRIANGLES); break;
      case 10:
        if (AppTouch_Ready()) snprintf(line, sizeof(line), "touch GT%s @%02X %s", AppTouch_ProductId(), AppTouch_Address(), ui_touch.touched ? "DOWN" : "UP");
        else snprintf(line, sizeof(line), "%s", TR("touch: not found"));
        break;
      default: snprintf(line, sizeof(line), "x=%u y=%u pts=%u i2c_err=%lu", ui_touch.x, ui_touch.y, ui_touch.points, (unsigned long)ui_touch.errors); break;
    }
    SmallUi_Text(SS_ROW(row), 12U, (uint16_t)(46U + row * 18U), 37U, line,
                 row == 8U && r->over ? S_ORANGE : row == 10U && !AppTouch_Ready() ? S_ORANGE : S_TEXT, S_CARD);
  }
  return drawn;
}

/* ---- Diagnostics -------------------------------------------------------------------------------- */
static void SmallUi_DiagLine(unsigned card, unsigned row, uint16_t color, const char *format, ...)
{
  char line[64];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  const uint16_t x = (uint16_t)(12U + (card % 2U) * 238U), y = (uint16_t)(64U + (card / 2U) * 116U + row * 20U);
  SmallUi_Text(SS_ROW(card * 4U + row), x, y, 27U, line, color, S_CARD);
}

static void SmallUi_DrawDiagnostics(const AppTasksStatusSnapshot *s, const InfrabotSafetyStatus *stop, uint8_t enter)
{
  static const char *const titles[4] = {"CONTROL", "CAN", "NETWORK", "DISPLAY"};
  if (enter)
    for (unsigned i = 0U; i < 4U; ++i)
      SmallUi_Card((uint16_t)(4U + (i % 2U) * 238U), (uint16_t)(40U + (i / 2U) * 116U), 234U, 112U, SmallUi_Tr(titles[i]));
  AppGravityStatus g;
  AppGravity_Status(&g);
  SmallUi_DiagLine(0U, 0U, s->control_missed ? S_RED : S_TEXT, TR("%lu Hz  miss %lu"), (unsigned long)s->control_hz,
                   (unsigned long)s->control_missed);
  SmallUi_DiagLine(0U, 1U, S_TEXT, TR("period max %lu us"), (unsigned long)s->period_max_us);
  SmallUi_DiagLine(0U, 2U, S_TEXT, TR("model calc %lu/%lu us"), (unsigned long)g.last_us, (unsigned long)g.max_us);
  const char *phase = stop->stop_phase == INFRABOT_STOP_PHASE_BRAKING ? TR("BRAKING") :
                      stop->stop_phase == INFRABOT_STOP_PHASE_HOLDING ? TR("HOLDING") : "";
  SmallUi_DiagLine(0U, 3U, stop->stop_reason ? S_ORANGE : S_TEXT, TR("STOP %s %s"),
                   SmallUi_Tr(AppLcdUi_StopReasonName(stop->stop_reason)), phase);

  SmallUi_DiagLine(1U, 0U, S_TEXT, "tx %lu/s  rx %lu/s", (unsigned long)s->can_tx, (unsigned long)s->can_rx);
  SmallUi_DiagLine(1U, 1U, s->can_tx_errors ? S_RED : S_TEXT, "txerr %lu  ev %lu", (unsigned long)s->can_tx_errors,
                   (unsigned long)s->can_error_events);
  SmallUi_DiagLine(1U, 2U, S_TEXT, "TEC %u  REC %u", s->can_tec, s->can_rec);
  SmallUi_DiagLine(1U, 3U, (s->can_bus_off || s->can_error_passive) ? S_RED : S_TEXT, "%s  %s",
                   s->can_error_passive ? "PASSIVE" : "ACTIVE", s->can_bus_off ? "BUS-OFF" : "BUS-OK");

  AppEthernetInfo info;
  AppEthernet_GetInfo(&info);
  const uint8_t link = AppEthernet_LinkUp();
  SmallUi_DiagLine(2U, 0U, link ? S_GREEN : S_ORANGE, TR("link %s"), link ? TR("UP") : TR("DOWN"));
  SmallUi_DiagLine(2U, 1U, S_TEXT, TR("cmd %lu/s  tel %lu/s"), (unsigned long)s->command_rx, (unsigned long)s->telemetry_tx);
  SmallUi_DiagLine(2U, 2U, s->command_invalid ? S_ORANGE : S_TEXT, TR("invalid %lu"), (unsigned long)s->command_invalid);
  if (info.client_valid)
  {
    char ip[20];
    AppLcdUi_FormatIp(ip, sizeof(ip), info.client_ip);
    SmallUi_DiagLine(2U, 3U, S_TEXT, "PC %s:%u", ip, info.client_port);
  }
  else SmallUi_DiagLine(2U, 3U, S_DIM, "%s", TR("PC none"));

  if (AppTouch_Ready()) SmallUi_DiagLine(3U, 0U, S_TEXT, TR("touch GT%s @%02X"), AppTouch_ProductId(), AppTouch_Address());
  else SmallUi_DiagLine(3U, 0U, S_ORANGE, "%s", TR("touch: not found"));
  SmallUi_DiagLine(3U, 1U, S_DIM, "%s x=%u y=%u pts=%u", ui_touch.touched ? "DOWN" : "UP", ui_touch.x, ui_touch.y,
                   ui_touch.points);
  SmallUi_DiagLine(3U, 2U, ui_touch.errors ? S_ORANGE : S_TEXT, "i2c_err %lu", (unsigned long)ui_touch.errors);
  SmallUi_DiagLine(3U, 3U, S_TEXT, TR("frame %lu/%lu us"), (unsigned long)ui_frame_us, (unsigned long)ui_frame_max_us);
}

/* ---- Settings ------------------------------------------------------------------------------------ */
static void SmallUi_InfoLine(unsigned row, uint16_t color, const char *format, ...)
{
  char line[160];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  SmallUi_Text(SS_ROW(row), 12U, (uint16_t)(76U + row * 16U), 57U, line, color, S_CARD);
}

/* LANGUAGE: applies at once; saved to flash only with every drive disabled,
 * because the sector erase blocks this task for about a second (as the
 * address save does).  Without a saved record the screen starts in English. */
enum { SL_NOTE_NONE, SL_NOTE_SAVED, SL_NOTE_ENABLED, SL_NOTE_BUSY, SL_NOTE_FAILED };
#define S_LANG_Y 102U
#define S_LANG_H 40U
#define S_LANG_X(k) (uint16_t)(12U + (k) * 232U)
#define S_LANG_W 224U

static void SmallUi_LoadLanguage(void)
{
  uint8_t language = APP_UI_LANGUAGE_EN;
  if (!AppPersist_LoadLanguage(&language) || language >= APP_UI_LANGUAGES) language = APP_UI_LANGUAGE_EN;
  small_ui.language = small_ui.language_saved = language;
  small_ui.language_note = SL_NOTE_NONE;
}

/* Returns 1 when the language changed (the page is drawn again). */
static uint8_t SmallUi_SetLanguage(uint8_t language)
{
  if (language >= APP_UI_LANGUAGES) return 0U;
  const uint8_t changed = language != small_ui.language;
  small_ui.language = language;
  small_ui.language_note = SL_NOTE_NONE;
  if (language == small_ui.language_saved) return changed;
  uint8_t enabled = dm_motor_runtime.control_enabled != 0U;
  for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i) enabled |= dm_motor_runtime.enabled[i] != 0U;
  if (enabled) small_ui.language_note = SL_NOTE_ENABLED;
  else if (AppPersist_Busy()) small_ui.language_note = SL_NOTE_BUSY;
  else
  {
    /* Shown before the blocking save. */
    SmallUi_Text(SS_ROW(2U), 12U, 170U, 57U, TR("Saving to MCU flash..."), S_YELLOW, S_CARD);
    if (AppPersist_SaveLanguage(language))
    {
      small_ui.language_saved = language;
      small_ui.language_note = SL_NOTE_SAVED;
    }
    else small_ui.language_note = SL_NOTE_FAILED;
  }
  return changed;
}

static void SmallUi_DrawLanguage(uint8_t enter)
{
  if (enter)
    for (uint8_t k = 0U; k < APP_UI_LANGUAGES; ++k)
      SmallUi_PutButton(S_LANG_X(k), S_LANG_Y, S_LANG_W, S_LANG_H, SA_LANGUAGE, k, small_language_names[k],
                        k == small_ui.language ? S_SELECT : S_BUTTON, S_TEXT);
  const uint8_t note = small_ui.language_note;
  SmallUi_Text(SS_ROW(0U), 12U, 80U, 57U, TR("Language of this 4.3in screen"), S_DIM, S_CARD);
  if (small_ui.language == small_ui.language_saved)
    SmallUi_Text(SS_ROW(1U), 12U, 152U, 57U, note == SL_NOTE_SAVED ? TR("Saved in MCU flash; kept after reboot") :
                 TR("Kept after reboot"), note == SL_NOTE_SAVED ? S_GREEN : S_DIM, S_CARD);
  else
    SmallUi_Text(SS_ROW(1U), 12U, 152U, 57U, TR("Not saved; the saved language returns after reboot"), S_ORANGE, S_CARD);
  const char *detail = note == SL_NOTE_ENABLED ? TR("Disable all motors, then tap it again to save.") :
                       note == SL_NOTE_BUSY ? TR("Flash busy; tap again") :
                       note == SL_NOTE_FAILED ? TR("Save failed (see UART log)") : "";
  SmallUi_Text(SS_ROW(2U), 12U, 170U, 57U, detail, note == SL_NOTE_FAILED ? S_RED : S_ORANGE, S_CARD);
  SmallUi_Text(SS_ROW(3U), 12U, 196U, 57U, TR("The 5in screen and PC tools stay in English."), S_DIM, S_CARD);
  SmallUi_Text(SS_ROW(4U), 12U, 214U, 57U, TR("Saving needs every motor disabled."), S_DIM, S_CARD);
  SmallUi_Text(SS_ROW(5U), 12U, 232U, 57U, TR("Do not power off while saving (about 1 s)."), S_DIM, S_CARD);
}

static void SmallUi_DrawSettings(uint8_t enter)
{
  if (enter) SmallUi_Card(4U, 72U, 472U, 196U, NULL);
  if (small_ui.view == 0U) { SmallUi_DrawLanguage(enter); return; }
  if (small_ui.view == 1U)
  {
    SmallUi_InfoLine(0U, S_TEXT, TR("MCU FW %s"), APP_FIRMWARE_VERSION);
    SmallUi_InfoLine(1U, S_DIM, TR("Build %s %s"), __DATE__, __TIME__);
    SmallUi_InfoLine(2U, S_TEXT, TR("Author YaolinDong"));
    SmallUi_InfoLine(3U, S_TEXT, TR("Email dong1505lin@gmail.com"));
    SmallUi_InfoLine(4U, S_TEXT, "MCU STM32H743 | CPU %lu MHz", (unsigned long)(SystemCoreClock / 1000000U));
    SmallUi_InfoLine(5U, S_TEXT, TR("LCD 4.3in %ux%u | UI %u Hz"), (unsigned)AppLcd_PhysicalWidth(),
                     (unsigned)AppLcd_PhysicalHeight(), (unsigned)DM_LCD_FRAME_RATE_HZ);
    SmallUi_InfoLine(6U, S_TEXT, TR("Control %u Hz | 8 motors"), (unsigned)DM_CONTROL_HZ);
    SmallUi_InfoLine(7U, dm_motor_runtime.profile_ready ? S_TEXT : S_ORANGE, TR("Profile %s"),
                     dm_motor_runtime.profile_ready ? TR("READY") : TR("NOT READY"));
    SmallUi_InfoLine(8U, S_DIM, TR("Firmware version = Git tag/revision (+dirty)"));
    SmallUi_InfoLine(9U, S_DIM, TR("Parameters are RAM only; the language is saved in flash"));
    SmallUi_InfoLine(10U, S_DIM, TR("Read only page"));
    SmallUi_InfoLine(11U, S_DIM, "");
    return;
  }
  if (small_ui.view == 2U)
  {
    SmallUi_InfoLine(0U, S_DIM, TR("Protocol encoding ranges, NOT software safety limits"));
    SmallUi_InfoLine(1U, S_DIM, "PMAX rad | VMAX rad/s | TMAX Nm");
    SmallUi_InfoLine(2U, S_ACCENT, TR("J  Model        ID/MST  PMAX  VMAX  TMAX"));
    for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i)
    {
      char p[12], v[12], t[12];
      AppLcdUi_Fixed(p, sizeof(p), dm_motor_position_max[i], 1U);
      AppLcdUi_Fixed(v, sizeof(v), dm_motor_velocity_max[i], 1U);
      AppLcdUi_Fixed(t, sizeof(t), dm_motor_torque_max[i], 1U);
      SmallUi_InfoLine(3U + i, S_TEXT, "%u  %-11s %02X/%02X %5s %5s %5s", i + 1U, dm_motor_names[i],
                       (unsigned)dm_motors[i].id, (unsigned)dm_motors[i].master_id, p, v, t);
    }
    SmallUi_InfoLine(11U, S_DIM, TR("J8: position/current"));
    return;
  }
  SmallUi_InfoLine(0U, S_DIM, TR("Kp Nm/rad | Kd Nm s/rad | APR drive gain"));
  SmallUi_InfoLine(1U, S_ACCENT, TR("J  Mode        Kp       Kd      KP_APR"));
  for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i)
  {
    char kp[12] = "--", kd[12] = "--", apr[12] = "--";
    if (dm_control_stats.task_started)
    {
      AppLcdUi_Fixed(kp, sizeof(kp), eth_udp_command.target_kp[i], 2U);
      AppLcdUi_Fixed(kd, sizeof(kd), eth_udp_command.target_kd[i], 2U);
      if (i < DM_ARM_MOTOR_COUNT) AppLcdUi_Fixed(apr, sizeof(apr), dm_posvel_kp_apr[i], 2U);
    }
    const char *mode = i == DM_GRIPPER_INDEX ? "POS/CUR" :
                       eth_udp_command.motor_mode[i] == DM_ETH_MODE_MIT ? "MIT" :
                       eth_udp_command.motor_mode[i] == DM_ETH_MODE_POS_VEL ? "POS/VEL" :
                       eth_udp_command.motor_mode[i] == DM_ETH_MODE_VEL ? "VEL" : "UNSET";
    SmallUi_InfoLine(2U + i, S_TEXT, "%u  %-8s %8s %8s %8s", i + 1U, mode, kp, kd, apr);
  }
  AppGravityParameters g;
  AppGravity_GetParameters(&g);
  char kp[12], kd[12];
  AppLcdUi_Fixed(kp, sizeof(kp), g.applied[APP_GRAVITY_STIFFNESS], 2U);
  AppLcdUi_Fixed(kd, sizeof(kd), g.applied[APP_GRAVITY_DAMPING], 2U);
  SmallUi_InfoLine(10U, S_TEXT, TR("Gravity applied: LOCK Kp %s | Kd %s"), kp, kd);
  SmallUi_InfoLine(11U, S_DIM, dm_control_stats.task_started ? TR("APR: cached/default; MIT Kp/Kd are not POS gains.") :
                                                         TR("Waiting for motor startup; profile not loaded"));
}

/* ---- Entry points (from app_lcd_ui.c when UI_SMALL) ------------------------------------------------ */
static void SmallUi_EnterPage(void)
{
  if (small_ui.previous_page != small_ui.page)
  {
    ui_window = (UiFrameWindow){0};
    ui_report = (UiFrameReport){0};
    ui_report_pending = 0U;
    ui_frame_us = ui_frame_max_us = ui_3d_us = ui_3d_max_us = 0U;
    small_ui.previous_page = small_ui.page;
  }
  ui_slider.index = APP_LCD_SLIDER_NONE;
  ui_touch.dragging = 0U;
  small_ui.count = 0U;
  small_ui.pressed = UINT8_MAX;
  small_ui.fresh = 1U;
  small_ui.health_drawn = small_ui.heartbeat_drawn = small_ui.bias_drawn = UINT8_MAX;
  small_ui.indicator_drawn[0] = small_ui.indicator_drawn[1] = UINT8_MAX;
  small_ui.param_drawn = UINT8_MAX;
  small_ui.knob_drawn = UINT16_MAX;
  memset(small_ui.tile_drawn, 0xFF, sizeof(small_ui.tile_drawn));
  memset(small_ui.dot_drawn, 0xFF, sizeof(small_ui.dot_drawn));
  for (unsigned i = 0U; i < UI_SLOT_COUNT; ++i) ui_slots[i].valid = 0U;
  AppLcd_FillRect(0U, 0U, AppLcd_PhysicalWidth(), AppLcd_PhysicalHeight(), S_BG);
  SmallUi_DrawTopStatic();
  SmallUi_DrawSegments();
  AppLcd3d_Invalidate();
}

static uint8_t SmallUi_DrawFrame(uint8_t heartbeat)
{
  AppTasksStatusSnapshot stat;
  AppTasks_GetStatusSnapshot(&stat);
  InfrabotSafetyStatus stop = {0};
  taskENTER_CRITICAL();
  AppSafety_Report(&stop);
  taskEXIT_CRITICAL();
  const uint8_t enter = small_ui.fresh;
  small_ui.fresh = 0U;
  SmallUi_DrawTopLive(&stat, stop.stop_reason, heartbeat);
  uint8_t rendered = 0U;
  switch (small_ui.page)
  {
    case SP_MENU:      SmallUi_DrawMenu(&stat, &stop, enter); break;
    case SP_ARM:       SmallUi_DrawArm(&stop, enter); break;
    case SP_3D:        rendered = SmallUi_Draw3d(enter); AppLcdUi_Report3d(); break;
    case SP_GRAVITY:   SmallUi_DrawGravity(enter); break;
    case SP_NETWORK:   SmallUi_DrawNetwork(enter); break;
    case SP_JOINT_IMP: SmallUi_DrawImpedance(0U, enter); break;
    case SP_CART_IMP:  SmallUi_DrawImpedance(1U, enter); break;
    case SP_JOINT_ADM: SmallUi_DrawAdmittance(0U, enter); break;
    case SP_CART_ADM:  SmallUi_DrawAdmittance(1U, enter); break;
    case SP_DIAG:      SmallUi_DrawDiagnostics(&stat, &stop, enter); break;
    default:           SmallUi_DrawSettings(enter); break;
  }
  return rendered;
}

/* The finger went down on button i.  Returns 1 when the page or view changed. */
static uint8_t SmallUi_Press(uint8_t i)
{
  const SmallButton b = small_ui.buttons[i];
  switch (b.action)
  {
    case SA_BACK:
      small_ui.page = SP_MENU;
      small_ui.view = 0U;
      return 1U;
    case SA_TILE:
      small_ui.page = b.arg;
      small_ui.view = 0U;
      small_ui.param = 0U;
      if (b.arg == SP_NETWORK)
      {
        AppLcdUi_NetLoadCurrent();
        AppLcdUi_NetMessage(UI_DIM, "", "");
      }
      return 1U;
    case SA_VIEW:
      if (b.arg == small_ui.view) return 0U;
      if (small_ui.page == SP_3D && b.arg == 1U) ui_3d_detail_report = ui_report;
      small_ui.view = b.arg;
      small_ui.param = 0U;
      return 1U;
    case SA_STOP:    AppLocalControl_Request(APP_LOCAL_STOP); break;
    case SA_ENABLE:  AppLocalControl_Request(APP_LOCAL_ENABLE); break;
    case SA_DISABLE: AppLocalControl_Request(APP_LOCAL_DISABLE); break;
    case SA_RESUME:  AppLocalControl_Request(APP_LOCAL_RESUME); break;
    case SA_ENTER:   AppLocalControl_Request(small_enter_action[small_ui.page]); break;
    case SA_FREE:    AppLocalControl_Request(APP_LOCAL_FREE); break;
    case SA_LOCK:    AppLocalControl_Request(APP_LOCAL_LOCK); break;
    case SA_BIAS:    AppAdmittance_SetBiasLearning(!AppAdmittance_BiasLearning()); break;
    case SA_VIEW3D:  AppLcd3d_SetView((AppLcd3dView)b.arg); break;
    case SA_LANGUAGE:
      if (SmallUi_SetLanguage(b.arg)) return 1U;
      break;
    case SA_KEY:
      if (b.arg < 16U)
      {
        const UiButton key = {0U, 0U, 0U, 0U, small_key_labels[b.arg], UI_PAGE_NETWORK, small_key_actions[b.arg], 0U};
        AppLcdUi_NetAction(&key);
      }
      break;
    default: break;
  }
  if (b.label[0] && i < small_ui.count)
  {
    small_ui.pressed = i;
    SmallUi_DrawButtonIndex(i, 1U);
  }
  return 0U;
}

/* Acts on the finger-down edge; a slider or 3D drag owns the gesture until
 * release.  Returns 1 when the page or view changed. */
static uint8_t SmallUi_HandleTouch(uint8_t was_touched)
{
  if (!ui_touch.touched)
  {
    ui_slider.index = APP_LCD_SLIDER_NONE;
    ui_touch.dragging = 0U;
    if (small_ui.pressed < small_ui.count) SmallUi_DrawButtonIndex(small_ui.pressed, 0U);
    small_ui.pressed = UINT8_MAX;
    return 0U;
  }
  if (ui_slider.index != APP_LCD_SLIDER_NONE) { SmallUi_ListDrag(); return 0U; }
  if (was_touched) return 0U;
  for (uint8_t i = 0U; i < small_ui.count; ++i)
  {
    const SmallButton *b = &small_ui.buttons[i];
    if (ui_touch.x >= b->x && ui_touch.x < b->x + b->w && ui_touch.y >= b->y && ui_touch.y < b->y + b->h)
      return SmallUi_Press(i);
  }
  if (small_ui.page == SP_3D && small_ui.view == 0U && ui_touch.x < S_3D_W && ui_touch.y >= S_3D_Y)
  {
    ui_touch.dragging = 1U;
    ui_touch.drag_x = ui_touch.x;
    ui_touch.drag_y = ui_touch.y;
    return 0U;
  }
  if (small_ui.page == SP_NETWORK && small_ui.view == 0U && ui_touch.x < 240U)
  {
    for (uint8_t i = 0U; i < 3U; ++i)
    {
      const uint16_t top = (uint16_t)(S_FIELD_Y(i) - 6U);
      if (ui_touch.y >= top && ui_touch.y < top + 26U) ui_net.selected = i;
    }
    return 0U;
  }
  SmallList list;
  if (SmallUi_PageList(&list)) SmallUi_ListPress(&list);
  return 0U;
}
