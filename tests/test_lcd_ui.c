/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Render the actual LCD page/handle actual touch gestures on a host framebuffer.
 * Optional argument: directory for HOME/GRAVITY PPM previews. No robot IO. */
#include "dm_motor.h"
#include "../app/app_tasks.h"
#include "../drivers/net/app_ethernet.h"
#define MOCK_APP_TASKS_H
#define MOCK_APP_ETHERNET_H
#define APP_GRAVITY_REAL_TEST
#include "app_gravity.h"
#include "app_local_control.h"
#include "app_lcd_font.h"
#include <assert.h>
uint32_t SystemCoreClock=480000000U;
#include "app_lcd_ui.c"

MockDwt lcd_dwt;
MockMotor dm_motors[8];

const char *const dm_motor_names[8]={"DM8009P-1","DM8009P-2","DM4340P-3","DM4340P-4","DM4310-5","DM4310-6","DM4310-7","DM4310-8"};
const float dm_motor_position_max[8]={12.5f,12.5f,12.5f,12.5f,12.5f,12.5f,12.5f,12.5f};
const float dm_motor_velocity_max[8]={45,45,10,10,30,30,30,30};
const float dm_motor_torque_max[8]={54,54,28,28,10,10,10,10};
static uint16_t pixels[480][800];

#ifdef TEST_LCD_43
#define TEST_PHYSICAL_W 480U
#define TEST_PHYSICAL_H 272U
#define TEST_MODEL 43U
#else
#define TEST_PHYSICAL_W 800U
#define TEST_PHYSICAL_H 480U
#define TEST_MODEL 5U
#endif
#ifdef FW_NETBOOT_APP
static FwPrompt test_request;
static unsigned choice;
int fw_consent_view(FwPrompt *p){*p=test_request;return p->id!=0;}
void fw_consent_decide(uint64_t id,int allow){
  assert(id && id==test_request.id);choice=allow?2:1;test_request.id=0;
}
static uint16_t labels[96*24];
uint16_t *AppLcd_Scratch(void){return labels;}
void SCB_CleanDCache_by_Addr(volatile void *p,int32_t size){assert(p==labels && size==sizeof(labels));}
void AppLcd_Blit(const uint16_t *src,uint16_t pitch,uint16_t x,uint16_t y,uint16_t w,uint16_t h){
  assert(x+w<=TEST_PHYSICAL_W && y+h<=TEST_PHYSICAL_H);
  for(unsigned j=0;j<h;j++)memcpy(&pixels[y+j][x],src+j*pitch,w*2U);
}
static void save(const char *dir,const char *name);
static void check_update_prompt(const char *dir){
 uint8_t redraw=0;
 for(unsigned allow=0;allow<2;allow++){
   test_request=(FwPrompt){.id=allow+1,.op=allow?FW_ENTER:FW_REVERT,.version="v0.2.0"};
   choice=0;ui_touch.points=1;ui_touch.touched=1;
   ui_touch.x=TEST_PHYSICAL_W/2+(allow?80:-80);ui_touch.y=TEST_PHYSICAL_H/2+40;
   assert(UpdateModal(0,&redraw) && UpdateModal(1,&redraw) && !choice);
   if(allow)save(dir,"update-prompt");
   ui_touch.touched=0;assert(UpdateModal(1,&redraw));
   ui_touch.touched=1;assert(UpdateModal(0,&redraw) && choice==allow+1);
   assert(UpdateModal(1,&redraw)); /* operator's finger is still down */
   ui_touch.touched=0;assert(!UpdateModal(1,&redraw) && redraw);
 }
 /* The longest reported version and no version (an older SDK) stay on screen. */
 static const char *const versions[]={"v12.345.6789-123-gabcdef","",};
 for(unsigned i=0;i<2;i++){
   test_request=(FwPrompt){.id=10+i,.op=FW_ENTER};
   snprintf(test_request.version,sizeof(test_request.version),"%s",versions[i]);
   ui_touch.touched=0;assert(UpdateModal(0,&redraw));
   save(dir,i?"update-prompt-noversion":"update-prompt-long");
   fw_consent_decide(test_request.id,0);assert(!UpdateModal(0,&redraw));
 }
}
#endif
static uint8_t test_touch_range_valid=1U;
static uint8_t test_touch_count;
static AppTouchPoint test_touch_point;
uint8_t AppLcd_PanelModel(void) { return TEST_MODEL; }
uint16_t AppLcd_PhysicalWidth(void) { return TEST_PHYSICAL_W; }
uint16_t AppLcd_PhysicalHeight(void) { return TEST_PHYSICAL_H; }
uint8_t AppTouch_Range(uint16_t *x,uint16_t *y) {
  *x=TEST_PHYSICAL_W; *y=TEST_PHYSICAL_H; return test_touch_range_valid;
}
uint8_t AppTouch_Read(AppTouchPoint p[APP_TOUCH_MAX_POINTS],uint8_t *n) {
  *n=test_touch_count; p[0]=test_touch_point; return 1U;
}
static unsigned drawing, actions;
static uint8_t last_action;
static AppGravityFault last_gravity_fault;
static uint8_t speed_limited_mask;
static ArmImpedanceDiagnostics mock_imp_diagnostics;
static uint16_t mock_entry_permille=1000U;
static uint8_t mock_recovering;
static AppLocalControlStatus mock_local_control;
static AppAdmittanceStatus mock_admittance;
void AppAdmittance_Status(AppAdmittanceStatus *out){*out=mock_admittance;}
static uint8_t mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;
const char *AppGravity_CheckName(uint8_t check) {
  return check==APP_GRAVITY_CHECK_SPEED ? "SPEED" : "UNKNOWN";
}
static float values[4]={.8f,8.f,0.f,.15f};
static const float mins[2]={.1f,.5f},maxs[2]={3,30};
float AppGravity_ParameterMin(uint8_t i) { return mins[i]; }
float AppGravity_ParameterMax(uint8_t i) { return maxs[i]; }
uint8_t AppGravity_SetParameter(uint8_t i,float value) {
  assert(i<2 && isfinite(value) && value>=mins[i] && value<=maxs[i]); values[i]=value; return 1;
}
void AppGravity_GetParameters(AppGravityParameters *out) {
  memcpy(out->value,values,sizeof(out->value)); memcpy(out->applied,values,sizeof(out->applied));
}
void AppGravity_Status(AppGravityStatus *s) {
  *s=(AppGravityStatus){.mode=mock_compliance_mode,.entry_permille=mock_entry_permille,.recovering=mock_recovering,.last_us=25,.max_us=26};
  s->torque[1]=4.285f; s->torque[3]=2.714f;
  s->impedance=mock_imp_diagnostics;
  s->fault=last_gravity_fault;
  s->speed_limited_mask=speed_limited_mask;
}
const char *ArmGravity_ModelName(void) { return "H7C bare v1"; }
const char *ArmGravity_ModelHash(void) { return "157078479caa4648dda4e07e3892a70aa4acccda68edc93e2da87e35cf16b29a"; }
void AppLocalControl_Request(uint8_t a) { ++actions; last_action=a; }
void AppLocalControl_Status(AppLocalControlStatus *s) { *s=mock_local_control; }
static uint8_t arm_waiting_mock, arm_responding_mock;
uint8_t AppStartup_ArmWaiting(uint8_t *n) { if(n) *n=arm_responding_mock; return arm_waiting_mock; }
const char *AppLocalControl_Message(const AppLocalControlStatus *s) {
  (void)s; return "J1-J7 controls; DISABLE removes holding torque";
}
static uint8_t mock_status=ROBOT_STATUS_IDLE_HOLD;
uint8_t RobotController_OperationalStatus(void) { return mock_status; }
const char *RobotController_StatusName(uint8_t status) { (void)status; return "IDLE_HOLD"; }
uint8_t AppSafety_TorqueLimitMask(void) { return 0; }
static InfrabotSafetyStatus mock_safety;
void AppSafety_Report(InfrabotSafetyStatus *s) { *s=mock_safety; }
uint8_t AppTouch_Ready(void) { return 1; }
const char *AppTouch_ProductId(void) { return "1158"; }
uint8_t AppTouch_Address(void) { return 0x28; }
static AppEthernetInfo mock_eth;
uint8_t AppEthernet_LinkUp(void) { return mock_eth.link_up; }
void AppEthernet_GetInfo(AppEthernetInfo *info) { *info=mock_eth; }
void AppEthernet_RequestAddress(const AppNetConfig *config) { (void)config; }
static uint8_t test_network_valid;
uint8_t AppPersist_NetworkValid(const AppNetConfig *config) { (void)config; return test_network_valid; }
uint8_t AppPersist_SaveNetwork(const AppNetConfig *config) { (void)config; return 0; }
/* Language record of the flash sector. */
static uint8_t mock_language_valid, mock_language, mock_flash_busy, mock_save_ok=1;
static unsigned language_saves;
uint8_t AppPersist_Busy(void) { return mock_flash_busy; }
uint8_t AppPersist_LoadLanguage(uint8_t *language) { if(mock_language_valid) *language=mock_language; return mock_language_valid; }
uint8_t AppPersist_SaveLanguage(uint8_t language) {
  assert(language<APP_UI_LANGUAGES && !mock_flash_busy && !dm_motor_runtime.control_enabled);
  for(unsigned i=0;i<8;++i) assert(!dm_motor_runtime.enabled[i]);
  ++language_saves;
  if(!mock_save_ok) return 0;
  mock_language=language;mock_language_valid=1;return 1;
}
const char *AppTasks_MotorStateName(uint8_t state) { return state==0xAU ? "OVERCURRENT" : "ENABLED"; }
void AppLcd3d_Invalidate(void) {}
void AppLcd3d_Orbit(int16_t x,int16_t y) { (void)x; (void)y; }
void AppLcd3d_SetView(AppLcd3dView view) { (void)view; }
void AppLcd_FillRect(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t color) {
  assert((unsigned)x+w<=TEST_PHYSICAL_W && (unsigned)y+h<=TEST_PHYSICAL_H); ++drawing;
  for(unsigned r=y;r<(unsigned)y+h;++r) for(unsigned c=x;c<(unsigned)x+w;++c) pixels[r][c]=color;

}
void AppLcd_DrawChar(uint16_t x,uint16_t y,char c,uint8_t size,uint16_t fg,uint16_t bg) {
  assert((unsigned)x+size/2<=TEST_PHYSICAL_W && (unsigned)y+size<=TEST_PHYSICAL_H); ++drawing;
  if(c<' ' || c>'~') c='?';
  const uint8_t *g=size==24 ? app_lcd_font24[c-' '] : app_lcd_font16[c-' '];

  for(unsigned r=0;r<size;++r) for(unsigned col=0;col<size/2;++col)
    pixels[y+r][x+col]=(g[col*((size+7)/8)+r/8] & (0x80U>>(r%8))) ? fg : bg;
}
void AppLcd_DrawText(uint16_t x,uint16_t y,const char *s,uint8_t size,uint16_t fg,uint16_t bg) {
  while(*s && x+size/2<=TEST_PHYSICAL_W) { AppLcd_DrawChar(x,y,*s++,size,fg,bg); x+=size/2; }
}
/* Same blend as app_lcd.c; a character without a glyph is a test failure. */
static unsigned wide_drawn;
void AppLcd_DrawWide(uint16_t x,uint16_t y,uint16_t code,uint16_t fg,uint16_t bg) {
  assert((unsigned)x+16<=TEST_PHYSICAL_W && (unsigned)y+16<=TEST_PHYSICAL_H); ++drawing; ++wide_drawn;
  const uint8_t *g=AppLcdZh_Glyph(code);
  if(!g) { fprintf(stderr,"no glyph for U+%04X\n",code); assert(!"missing glyph"); }
  uint16_t shade[16];AppLcdZh_Palette(fg,bg,shade);
  for(unsigned r=0;r<16;++r) for(unsigned c=0;c<16;++c)
    pixels[y+r][x+c]=shade[(g[r*8+c/2]>>(c%2?0:4))&15];
}
static void save(const char *dir,const char *name) {
  if(!dir) return;
  char path[1024]; snprintf(path,sizeof(path),"%s/%s.ppm",dir,name);
  FILE *out=fopen(path,"wb"); assert(out); fprintf(out,"P6\n%u %u\n255\n",TEST_PHYSICAL_W,TEST_PHYSICAL_H);
  for(unsigned r=0;r<TEST_PHYSICAL_H;++r) for(unsigned c=0;c<TEST_PHYSICAL_W;++c) {
    uint16_t p=pixels[r][c];
    const unsigned char rgb[3]={(unsigned char)((p>>11)*255/31),
      (unsigned char)(((p>>5)&63)*255/63),(unsigned char)((p&31)*255/31)};
    assert(fwrite(rgb,1,3,out)==3);
  }
  fclose(out);

}
#ifndef TEST_LCD_43
static void touch(uint8_t down,uint16_t x,uint16_t y) {
  const uint8_t was=ui_touch.touched;
  ui_touch.touched=down; ui_touch.x=x; ui_touch.y=y;
  if(AppLcdUi_HandleTouch(was)) AppLcdUi_EnterPage();
}
static void raw_touch(uint8_t down,uint16_t x,uint16_t y) {
  const uint8_t was=ui_touch.touched;
  test_touch_count=down;
  test_touch_point=(AppTouchPoint){x,y};
  AppLcdUi_PollTouch();
  if(AppLcdUi_HandleTouch(was)) AppLcdUi_EnterPage();
}
static void mapped_touch(uint8_t down,uint16_t x,uint16_t y) {
  raw_touch(down,x,y);
}
static void tap_action(uint8_t a) {
  for(unsigned i=0;i<UI_BUTTON_COUNT;++i) if(ui_buttons[i].action==a && AppLcdUi_ButtonVisible(i)) {
    const UiButton b=ui_buttons[i];mapped_touch(1,b.x+b.w/2,b.y+b.h/2);
    mapped_touch(0,b.x+b.w/2,b.y+b.h/2);return;
  }
  assert(!"missing button");
}
static void test_admittance_ui(const char *dir){
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;mock_local_control=(AppLocalControlStatus){0};
  tap_action(UI_ACT_TAB_ADMITTANCE);assert(ui_page==UI_PAGE_ADMITTANCE);
  AppLcdUi_DrawAdmittancePage();assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"NOT IN"));
  tap_action(UI_ACT_ADM_JOINT);assert(last_action==APP_LOCAL_JOINT_ADM);
  mock_local_control.pending_actions=1U<<APP_LOCAL_JOINT_ADM;
  AppLcdUi_DrawAdmittancePage();assert(ui_adm_indicator[0]==1 && strstr(ui_slots[UI_SLOT_DETAIL0].text,"WAITING"));
  mock_local_control.pending_actions=0;mock_compliance_mode=DM_ETH_CONTROL_JOINT_ADMITTANCE;mock_entry_permille=200;
  AppLcdUi_DrawAdmittancePage();assert(ui_adm_indicator[0]==2 && strstr(ui_slots[UI_SLOT_DETAIL0].text,"RAMP"));
  mock_entry_permille=1000;AppLcdUi_DrawAdmittancePage();assert(ui_adm_indicator[0]==3);
  save(dir,"lcd-adm-joint");
  for(unsigned group=0;group<6;group++){
    assert(ui_detail[UI_PAGE_ADMITTANCE]==group);AppLcdUi_DrawAdmittancePage();
    if(group==3){
      assert(AppAdmittance_BiasLearning());tap_action(UI_ACT_ADM_BIAS);
      assert(!AppAdmittance_BiasLearning());tap_action(UI_ACT_ADM_BIAS);
      assert(AppAdmittance_BiasLearning());
    }
    if(group==2)save(dir,"lcd-adm-mass-zeta");
    AppLcdSlider sl[2];assert(AdmittanceSliders(sl)==2);
    for(unsigned i=0;i<2;i++){
      unsigned parameter=adm_group[group][i];AppAdmittanceParameters before,after;AppAdmittance_GetParameters(&before);
      touch(1,sl[i].x+sl[i].width,sl[i].y+46);AppAdmittance_GetParameters(&after);
      assert(after.requested[parameter]==AppAdmittance_ParameterMax(parameter));
      for(unsigned j=0;j<APP_ADM_PARAMETERS;j++)if(j!=parameter)assert(after.requested[j]==before.requested[j]);
      assert(after.applied[parameter]==before.applied[parameter]);
      unsigned count=actions;touch(1,350,120);touch(1,40,10);assert(actions==count && ui_page==UI_PAGE_ADMITTANCE);
      touch(1,0,sl[i].y+46);AppAdmittance_GetParameters(&after);
      assert(after.requested[parameter]==AppAdmittance_ParameterMin(parameter));touch(0,0,sl[i].y+46);
    }
    tap_action(UI_ACT_GRAV_STOP);assert(last_action==APP_LOCAL_STOP);
    tap_action(UI_ACT_TAB_ADMITTANCE);
  }
  assert(ui_detail[UI_PAGE_ADMITTANCE]==6);tap_action(UI_ACT_ADM_CART);assert(last_action==APP_LOCAL_CART_ADM);
  mock_compliance_mode=DM_ETH_CONTROL_CARTESIAN_ADMITTANCE;
  mock_admittance=(AppAdmittanceStatus){.mode=DM_ETH_CONTROL_CARTESIAN_ADMITTANCE,.force={1,2,3},.moment={.1f,.2f,.3f},
    .error={.04f,.02f,.01f},.external={.12f},.tracking={.002f}};
  AppLcdUi_DrawAdmittancePage();assert(ui_adm_indicator[1]==3 && strstr(ui_slots[UI_SLOT_GRAV0].text,"1.0"));
  save(dir,"lcd-adm-diagnostics");
  AppAdmittanceParameters before,after;AppAdmittance_GetParameters(&before);touch(1,200,150);touch(0,200,150);
  AppAdmittance_GetParameters(&after);assert(!memcmp(&before,&after,sizeof(before)));
  mock_admittance.frozen=1;AppLcdUi_DrawAdmittancePage();assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"FROZEN"));
  mock_admittance.frozen=0;mock_admittance.returning=1;AppLcdUi_DrawAdmittancePage();assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"RETURN"));
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;mock_local_control=(AppLocalControlStatus){.action=APP_LOCAL_CART_ADM,.result=7};
  AppLcdUi_DrawAdmittancePage();assert(strstr(ui_slots[UI_SLOT_DETAIL0+1].text,"Rejected"));
  mock_local_control=(AppLocalControlStatus){0};mock_admittance=(AppAdmittanceStatus){0};
  mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;
  tap_action(UI_ACT_TAB_GRAVITY);
}
static void test_impedance_ui(const char *dir) {
  tap_action(UI_ACT_TAB_GRAVITY);
  if(ui_detail[UI_PAGE_GRAVITY]) tap_action(UI_ACT_TAB_GRAVITY);
  for(unsigned i=0;i<UI_BUTTON_COUNT;i++)
    if(ui_buttons[i].action==UI_ACT_GRAV_JOINT_IMP || ui_buttons[i].action==UI_ACT_GRAV_CART_IMP)
      assert(!AppLcdUi_ButtonVisible(i));
  const float gravity_k=values[1],gravity_d=values[0];
  tap_action(UI_ACT_TAB_IMPEDANCE);assert(ui_page==UI_PAGE_IMPEDANCE);
  tap_action(UI_ACT_GRAV_JOINT_IMP);assert(last_action==APP_LOCAL_JOINT_IMP);
  mock_compliance_mode=DM_ETH_CONTROL_JOINT_IMPEDANCE;
  AppLcdUi_DrawImpedancePage();save(dir,"lcd-joint-impedance");
  assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"JOINT IMP"));
  for(unsigned group=0;group<4;group++) {
    assert(ui_detail[UI_PAGE_IMPEDANCE]==group);
    AppLcdUi_DrawImpedancePage();
    if(group==1)save(dir,"lcd-cartesian-impedance");
    if(group==2)save(dir,"lcd-rotation-impedance");
    if(group==3)save(dir,"lcd-damping-impedance");
    AppLcdSlider sl[2];const unsigned count=ImpedanceSliders(sl);
    for(unsigned i=0;i<count;i++) {
      unsigned parameter=imp_group[group][i];
      AppImpedanceParameters before,after;AppImpedance_GetParameters(&before);
      touch(1,sl[i].x+sl[i].width,sl[i].y+46);
      AppImpedance_GetParameters(&after);
      assert(after.requested[parameter]==AppImpedance_ParameterMax(parameter));
      for(unsigned j=0;j<APP_IMP_PARAMETERS;j++) if(j!=parameter)
        assert(after.requested[j]==before.requested[j]);
      assert(after.applied[parameter]==before.applied[parameter]);
      ArmImpedanceGains applied;AppImpedance_Tick(.001f,&applied);
      AppImpedance_GetParameters(&after);
      assert(after.applied[parameter]>before.applied[parameter] && after.applied[parameter]<after.requested[parameter]);
      touch(1,0,sl[i].y+46);AppImpedance_GetParameters(&after);
      assert(after.requested[parameter]==AppImpedance_ParameterMin(parameter));
      unsigned action_before=actions;
      /* Captured drag cannot trigger a mode switch/navigation/STOP. */
      touch(1,350,120);assert(actions==action_before);
      touch(1,40,10);assert(ui_page==UI_PAGE_IMPEDANCE);
      touch(0,40,10);assert(ui_imp_slider.index==APP_LCD_SLIDER_NONE);
    }
    tap_action(UI_ACT_GRAV_STOP);assert(last_action==APP_LOCAL_STOP);
    tap_action(UI_ACT_TAB_IMPEDANCE);
    if(group==0) {tap_action(UI_ACT_GRAV_CART_IMP);assert(last_action==APP_LOCAL_CART_IMP);mock_compliance_mode=DM_ETH_CONTROL_CARTESIAN_IMPEDANCE;}
  }
  assert(ui_detail[UI_PAGE_IMPEDANCE]==4);
  mock_imp_diagnostics=(ArmImpedanceDiagnostics){.valid=1,.target={.3f,-.2f,.45f},
    .measured={.3f,-.2f,.35f},.position_error={0,0,.1f},.force={0,0,8}};
  AppLcdUi_DrawImpedancePage();save(dir,"lcd-return-diagnostics");
  assert(strstr(ui_slots[UI_SLOT_GRAV0].text,"450.0"));
  assert(strstr(ui_slots[UI_SLOT_GRAV0+1].text,"350.0"));
  assert(strstr(ui_slots[UI_SLOT_GRAV0+2].text,"100.0"));
  AppImpedanceParameters before_diag,after_diag;AppImpedance_GetParameters(&before_diag);
  touch(1,200,150);touch(0,200,150);AppImpedance_GetParameters(&after_diag);
  assert(!memcmp(&before_diag,&after_diag,sizeof(before_diag)));
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;AppLcdUi_DrawImpedancePage();
  assert(strstr(ui_slots[UI_SLOT_GRAV0].text,"No live"));
  assert(!ui_slots[UI_SLOT_GRAV0+2].text[0]);
  mock_imp_diagnostics.valid=0;
  tap_action(UI_ACT_TAB_IMPEDANCE);assert(ui_detail[UI_PAGE_IMPEDANCE]==0);
  assert(values[0]==gravity_d && values[1]==gravity_k);
  mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;
  tap_action(UI_ACT_TAB_GRAVITY);AppLcdUi_DrawGravityPage();
}
static void test_impedance_confirmation(const char *dir) {
  mock_local_control=(AppLocalControlStatus){0};mock_compliance_mode=DM_ETH_CONTROL_IDLE;
  tap_action(UI_ACT_TAB_IMPEDANCE);AppLcdUi_DrawImpedancePage();
  tap_action(UI_ACT_GRAV_JOINT_IMP);AppLcdUi_DrawImpedancePage();
  assert(!ui_imp_indicator[0] && !ui_imp_indicator[1]);
  assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"NOT IN IMPEDANCE"));
  mock_local_control.pending_actions=1U<<APP_LOCAL_JOINT_IMP;AppLcdUi_DrawImpedancePage();
  assert(ui_imp_indicator[0]==1U && ui_slots[UI_SLOT_DETAIL0].fg==UI_YELLOW);
  assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"WAITING"));
  mock_local_control=(AppLocalControlStatus){.action=APP_LOCAL_JOINT_IMP,.result=7};
  AppLcdUi_DrawImpedancePage();assert(!ui_imp_indicator[0]);
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+1].text,"JOINT rejected"));
  save(dir,"lcd-imp-rejected");
  mock_local_control.result=0;mock_compliance_mode=DM_ETH_CONTROL_JOINT_IMPEDANCE;
  mock_entry_permille=200;AppLcdUi_DrawImpedancePage();
  assert(ui_imp_indicator[0]==2U);
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+1].text,"20%"));save(dir,"lcd-imp-ramp");
  mock_entry_permille=1000;AppLcdUi_DrawImpedancePage();
  assert(ui_imp_indicator[0]==3U && ui_slots[UI_SLOT_DETAIL0].fg==UI_GREEN);
  assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"ACTIVE: JOINT IMPEDANCE"));
  assert(!strstr(ui_slots[UI_SLOT_DETAIL0+1].text,"Last stop")); /* old fault does not obscure success */
  save(dir,"lcd-imp-active");
  unsigned painted=drawing;AppLcdUi_DrawImpedancePage();assert(drawing==painted);
  /* Failed CART request leaves the confirmed JOINT mode highlighted. */
  mock_local_control=(AppLocalControlStatus){.action=APP_LOCAL_CART_IMP,.result=6};
  AppLcdUi_DrawImpedancePage();assert(ui_imp_indicator[0]==3U && !ui_imp_indicator[1]);
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+1].text,"CART rejected"));
  /* Entering the page via another tab and external mode changes also refresh. */
  mock_compliance_mode=DM_ETH_CONTROL_CARTESIAN_IMPEDANCE; /* old local rejection, external success */
  tap_action(UI_ACT_TAB_INFO);tap_action(UI_ACT_TAB_IMPEDANCE);AppLcdUi_DrawImpedancePage();
  assert(!ui_imp_indicator[0] && ui_imp_indicator[1]==3U);
  assert(!strstr(ui_slots[UI_SLOT_DETAIL0+1].text,"rejected"));
  for(unsigned i=0;i<UI_BUTTON_COUNT;i++)if(ui_buttons[i].action==UI_ACT_GRAV_CART_IMP) {
    const UiButton *b=&ui_buttons[i];assert(pixels[b->y+1][b->x+1]==UI_BUTTON_GO);
  }
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;mock_recovering=1;mock_entry_permille=0;
  AppLcdUi_DrawImpedancePage();assert(!ui_imp_indicator[0] && !ui_imp_indicator[1]);
  assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"STOPPING"));
  mock_recovering=0;AppLcdUi_DrawImpedancePage();
  assert(strstr(ui_slots[UI_SLOT_DETAIL0].text,"NOT IN IMPEDANCE"));
  mock_entry_permille=1000;mock_local_control=(AppLocalControlStatus){0};
}
static void test_info(const char *dir) {
  unsigned actions_before=actions;
  tap_action(UI_ACT_TAB_INFO);assert(ui_page==UI_PAGE_INFO && ui_detail[ui_page]==0);
  AppLcdUi_DrawInfoPage();save(dir,"lcd-info-about");
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+1].text,APP_FIRMWARE_VERSION));
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+4].text,"dong1505lin@gmail.com"));
  tap_action(UI_ACT_TAB_INFO);assert(ui_detail[ui_page]==1);
  for(unsigned i=0;i<8;i++){dm_motors[i].id=i+1;dm_motors[i].master_id=i+17;}
  AppLcdUi_DrawInfoPage();save(dir,"lcd-info-motors");
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+3].text,"DM8009P-1  "));
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+10].text,"08/18"));
  tap_action(UI_ACT_TAB_INFO);assert(ui_detail[ui_page]==2);
  dm_control_stats.task_started=0;AppLcdUi_DrawInfoPage();
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+3].text,"--"));
  dm_control_stats.task_started=1;
  for(unsigned i=0;i<8;i++) {
    eth_udp_command.target_kp[i]=123.45f;eth_udp_command.target_kd[i]=1.25f;
    dm_posvel_kp_apr[i]=54;eth_udp_command.motor_mode[i]=DM_ETH_MODE_MIT;
  }
  eth_udp_command.motor_mode[1]=DM_ETH_MODE_POS_VEL;
  AppLcdUi_DrawInfoPage();save(dir,"lcd-info-pd");
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+3].text,"123.45"));
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+4].text,"POS/VEL"));
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+10].text,"POS/CUR"));
  eth_udp_command.target_kp[0]=99.5f;AppLcdUi_DrawInfoPage();
  assert(strstr(ui_slots[UI_SLOT_DETAIL0+3].text,"99.50"));
  touch(1,200,140);touch(0,200,140);assert(actions==actions_before);
  tap_action(UI_ACT_TAB_INFO);assert(ui_detail[ui_page]==0);
  tap_action(UI_ACT_TAB_STATUS);assert(ui_page==UI_PAGE_STATUS);
}
int main(int argc,char **argv) {
  const char *dir=argc>1 ? argv[1] : NULL;
#ifdef FW_NETBOOT_APP
  check_update_prompt(dir);
#endif
  dm_motor_runtime.control_enabled=1;
  for(unsigned i=0;i<7;++i) {
    dm_motor_runtime.enabled[i]=1; dm_motors[i].feedback_count=1;
    dm_motors[i].temperature_mos=39; dm_motors[i].state=1;
    dm_motor_runtime.command_position[i]=dm_motors[i].position=.05f;
  }
  AppTasksStatusSnapshot stat={.control_hz=1000,.can_tx=7000,.can_rx=7000,.period_max_us=1000};
  assert(ui_page==UI_PAGE_STATUS);
  AppLcdUi_EnterPage(); AppLcdUi_DrawNav(1); AppLcdUi_DrawStatusPage(&stat);
  save(dir,"lcd-home");
  char torque[20]; AppLcdUi_Fixed(torque,sizeof(torque),4.285f,3);
  assert(!strcmp(torque,"4.285"));
  for(unsigned i=0;i<4;++i) {
    touch(1,(uint16_t)(100+i*196),78); assert(actions==i+1 && last_action==i);
    touch(1,(uint16_t)(100+i*196),79); assert(actions==i+1); /* no repeat while held */
    touch(0,0,0);
  }
  tap_action(UI_ACT_TAB_GRAVITY); assert(ui_page==UI_PAGE_GRAVITY);
  AppLcdUi_DrawGravityPage(); save(dir,"lcd-gravity");
  unsigned before=drawing; AppLcdUi_DrawGravityPage(); assert(drawing==before); /* unchanged => no repaint */
  touch(1,280,230); assert(values[0]>1.5f && values[0]<1.6f); /* mid-slider of 0.1..3 */
  touch(1,799,280); assert(values[0]==3.f && values[1]==8.f); /* drag capture */
  touch(1,700,120); assert(actions==4); /* dragging onto STOP does not press it */
  touch(0,0,0); touch(1,700,120); assert(actions==5 && last_action==APP_LOCAL_STOP);
  touch(0,0,0);
  for(unsigned i=0;i<APP_GRAVITY_PARAMETERS;++i) {
    touch(1,520,(uint16_t)(230+52*i)); assert(values[i]==maxs[i]);
    touch(1,0,(uint16_t)(230+52*i)); assert(values[i]==mins[i]);
    touch(0,0,0);
  }
  AppLcdUi_DrawGravityPage();
  last_gravity_fault=(AppGravityFault){.sequence=1,.check=APP_GRAVITY_CHECK_SPEED,
                                      .joint=2,.value=3.2f,.limit=3.14f};
  AppLcdUi_DrawGravityPage();
  assert(strstr(ui_slots[UI_SLOT_GRAV0+2].text,"Last stop J3 SPEED: 3.200 / 3.140 rad/s"));
  speed_limited_mask=(1U<<2)|(1U<<5);
  AppLcdUi_DrawGravityPage();
  assert(strstr(ui_slots[UI_SLOT_GRAV0+10].text,"Braking (speed/joint limit): J3 J6"));
  speed_limited_mask=0;
  AppLcdUi_DrawGravityPage();
  assert(!strstr(ui_slots[UI_SLOT_GRAV0+10].text,"Braking"));
  touch(1,70,20); touch(0,70,20); assert(ui_page==UI_PAGE_STATUS);
  AppLcdUi_DrawStatusPage(&stat);
  arm_waiting_mock=1; arm_responding_mock=0;      /* arm powered after the MCU */
  AppLcdUi_DrawStatusPage(&stat);
  assert(strstr(ui_slots[UI_SLOT_LOCAL].text,"Waiting for arm power: 0/7"));
  arm_waiting_mock=0;
  AppLcdUi_DrawStatusPage(&stat);
  assert(!strstr(ui_slots[UI_SLOT_LOCAL].text,"Waiting"));
  ui_page=UI_PAGE_NETWORK;AppLcdUi_EnterPage();AppLcdUi_DrawNav(1);
  AppLcdUi_DrawNetworkPage();save(dir,"lcd-network");
  /* Exercise real raw-touch -> UI coordinate conversion, not just hit tests. */
  mapped_touch(1,70,20);mapped_touch(0,70,20);assert(ui_page==UI_PAGE_STATUS);
  unsigned action_before=actions;
  mapped_touch(1,500,78);assert(last_action==APP_LOCAL_STOP && actions==action_before+1);
  mapped_touch(0,500,78);
  test_touch_range_valid=0;
  mapped_touch(1,100,78);assert(actions==action_before+1); /* unknown range cannot enable */
  mapped_touch(0,100,78);test_touch_range_valid=1;
  raw_touch(1,TEST_PHYSICAL_W,0);assert(!ui_touch.touched);raw_touch(0,0,0);
  tap_action(UI_ACT_TAB_NETWORK);assert(ui_page==UI_PAGE_NETWORK);
  const size_t len_before=strlen(ui_net.field[ui_net.selected]);
  mapped_touch(1,490,80);mapped_touch(0,490,80);
  assert(strlen(ui_net.field[ui_net.selected])==len_before+1U);
  test_admittance_ui(dir);
  test_impedance_ui(dir);
  test_impedance_confirmation(dir);
  test_info(dir);
  puts("LCD pages: drawing bounds, torque precision, home controls, touch capture, all sliders, cached redraw: OK");
}

#else
/* ---- 4.3in: main menu, top bar, sub-pages --------------------------------- */
static TickType_t mock_tick=1000U;
TickType_t xTaskGetTickCount(void) { return mock_tick; }
static unsigned model_renders;
uint8_t AppLcd3d_Render(uint16_t x,uint16_t y,uint16_t w,uint16_t h) {
  ++model_renders;
  assert(x==0 && y==S_3D_Y && w==S_3D_W && h==S_3D_H && (uint32_t)w*h<=APP_LCD_SCRATCH_PIXELS);
  AppLcd_FillRect(x,y,w,h,0x0842);return 1;
}
void AppLcd3d_Timing(AppLcd3dTiming *t) {
  *t=(AppLcd3dTiming){.prepare_cycles=480000,.draw_cycles=960000,.clear_cycles=48000,
    .raster_cycles=720000,.clean_cycles=24000,.blit_cycles=168000,.updated_pixels=39040,
    .sram_buffer=1,.tiles=4};
}
static AppTasksStatusSnapshot mock_stat={.control_hz=1000,.can_tx=7000,.can_rx=7000,.period_max_us=1000,
  .command_rx=1000,.telemetry_tx=1000};
void AppTasks_GetStatusSnapshot(AppTasksStatusSnapshot *s) { *s=mock_stat; }

/* One LCD task iteration: touch, page entry, frame. */
static uint8_t frame_heartbeat=1;
static void frame(void) { SmallUi_DrawFrame(frame_heartbeat); }
static void small_touch(uint8_t down,uint16_t x,uint16_t y) {
  const uint8_t was=ui_touch.touched;
  ui_touch.touched=down;ui_touch.x=x;ui_touch.y=y;ui_touch.points=down;
  if(AppLcdUi_HandleTouch(was)) AppLcdUi_EnterPage();
  frame();
}
static void tap_xy(uint16_t x,uint16_t y) { small_touch(1,x,y);small_touch(0,x,y); }
static const SmallButton *find(uint8_t action,uint8_t arg) {
  for(unsigned i=0;i<small_ui.count;++i)
    if(small_ui.buttons[i].action==action && small_ui.buttons[i].arg==arg) return &small_ui.buttons[i];
  return NULL;
}
static void tap(uint8_t action,uint8_t arg) {
  const SmallButton *b=find(action,arg);assert(b);
  tap_xy((uint16_t)(b->x+b->w/2),(uint16_t)(b->y+b->h/2));
}
static void open_page(uint8_t page) {
  if(small_ui.page!=SP_MENU) tap(SA_BACK,0);
  assert(small_ui.page==SP_MENU);
  tap(SA_TILE,page);assert(small_ui.page==page && small_ui.view==0);
}
static void view(uint8_t v) { tap(SA_VIEW,v);assert(small_ui.view==v); }
static int screen_has(const char *text) {
  for(unsigned i=0;i<UI_SLOT_COUNT;++i) if(ui_slots[i].valid && strstr(ui_slots[i].text,text)) return 1;
  return 0;
}
static uint16_t slot_color(const char *text) {
  for(unsigned i=0;i<UI_SLOT_COUNT;++i) if(ui_slots[i].valid && strstr(ui_slots[i].text,text)) return ui_slots[i].fg;
  assert(!"text not on screen");return 0;
}
/* Layout invariants every page must keep. */
static void check_layout(void) {
  const SmallButton *stop=find(SA_STOP,0);
  assert(stop && stop->x==S_STOP_X && stop->y==S_BAR_BTN_Y);           /* STOP on every page, same place */
  if(small_ui.page==SP_MENU) assert(!find(SA_BACK,0));
  else { const SmallButton *b=find(SA_BACK,0);assert(b && b->x==S_BACK_X && b->x+b->w+150<stop->x); }
  for(unsigned i=0;i<small_ui.count;++i) {
    const SmallButton *a=&small_ui.buttons[i];
    assert(a->x+a->w<=480 && a->y+a->h<=272 && a->w>=40 && a->h>=22);  /* fingertip size, on screen */
    assert(SmallUi_Width(a->label)+8U<=a->w);
    for(unsigned j=i+1;j<small_ui.count;++j) {                          /* no overlapping targets */
      const SmallButton *b=&small_ui.buttons[j];
      assert(a->x+a->w<=b->x || b->x+b->w<=a->x || a->y+a->h<=b->y || b->y+b->h<=a->y);
    }
  }
  for(unsigned i=0;i<UI_SLOT_COUNT;++i) if(ui_slots[i].valid) {
    assert(ui_slots[i].size==16);
    assert(ui_slots[i].x+ui_slots[i].length*8U<=480 && ui_slots[i].y+16U<=272);
    assert(AppLcdZh_Columns(ui_slots[i].text)<=ui_slots[i].length);
  }
  /* The page title (sub-pages) never runs into the state chip. */
  if(small_ui.page!=SP_MENU) assert(S_TITLE_X+SmallUi_Width(SmallUi_Tr(small_titles[small_ui.page]))<=S_STATE_X);
}
static void shot(const char *dir,const char *name) { check_layout();save(dir,name); }

static void test_menu(const char *dir) {
  AppLcdUi_EnterPage();frame();
  assert(small_ui.page==SP_MENU);
  for(unsigned i=0;i<10;++i) {
    const SmallButton *b=find(SA_TILE,small_menu_pages[i]);assert(b);
    assert(SmallUi_Width(small_titles[small_menu_pages[i]])+12U<=b->w);   /* full names fit */
  }
  assert(screen_has("Enabled 7/7") && screen_has("Spring on J1-J7") && screen_has("FW test-revision"));
  assert(!strcmp(ui_slots[SS_STATE].text,"IDLE HOLD") && small_ui.health_drawn==SH_OK);
  shot(dir,"lcd43-menu");
  /* Unchanged frame: nothing drawn. */
  unsigned before=drawing;frame();assert(drawing==before);
  /* Heartbeat toggles only the dot. */
  frame_heartbeat=0;before=drawing;frame();assert(drawing>before && drawing-before<20);frame_heartbeat=1;frame();
  /* An active mode lights its tile; a stop turns the arm tile orange and the health word red. */
  mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;mock_entry_permille=1000;frame();
  assert(small_ui.tile_drawn[2]==ST_LIVE && screen_has("FREE active"));
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;
  mock_status=ROBOT_STATUS_STOPPING;mock_safety.stop_reason=INFRABOT_STOP_STREAM_LOST;
  mock_safety.stop_phase=INFRABOT_STOP_PHASE_HOLDING;frame();
  assert(small_ui.tile_drawn[0]==ST_WARN && screen_has("STOP: STREAM_LOST"));
  assert(!strcmp(ui_slots[SS_STATE].text,"STOPPING") && small_ui.health_drawn==SH_PC_LOST);
  shot(dir,"lcd43-menu-stopped");
  /* STOP from the menu. */
  unsigned count=actions;tap(SA_STOP,0);assert(actions==count+1 && last_action==APP_LOCAL_STOP);
  mock_status=ROBOT_STATUS_IDLE_HOLD;mock_safety=(InfrabotSafetyStatus){0};
  mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;frame();
}

static void test_arm(const char *dir) {
  open_page(SP_ARM);
  assert(screen_has("J1") && screen_has("OK") && screen_has("NO FEEDBACK"));
  char xyz[3][16],rpy[3][16];SmallUi_Pose(xyz,rpy);assert(screen_has(xyz[2]) && screen_has(rpy[0]));
  shot(dir,"lcd43-arm");
  tap(SA_ENABLE,0);assert(last_action==APP_LOCAL_ENABLE);
  tap(SA_DISABLE,0);assert(last_action==APP_LOCAL_DISABLE);
  tap(SA_RESUME,0);assert(last_action==APP_LOCAL_RESUME);
  tap(SA_STOP,0);assert(last_action==APP_LOCAL_STOP);
  /* Pressed highlight while the finger is down; restored on release. */
  const SmallButton *e=find(SA_ENABLE,0);
  small_touch(1,(uint16_t)(e->x+e->w/2),(uint16_t)(e->y+e->h/2));
  assert(pixels[e->y+e->h/2][e->x+4]==S_PRESSED);
  small_touch(1,(uint16_t)(e->x+e->w/2+3),(uint16_t)(e->y+e->h/2));assert(last_action==APP_LOCAL_ENABLE);
  small_touch(0,0,0);assert(pixels[e->y+e->h/2][e->x+4]==S_GO);
  /* An unknown touch range cannot press anything. */
  unsigned count=actions;test_touch_range_valid=0;test_touch_count=1;
  test_touch_point=(AppTouchPoint){(uint16_t)(e->x+5),(uint16_t)(e->y+5)};
  AppLcdUi_PollTouch();assert(!ui_touch.touched);
  if(AppLcdUi_HandleTouch(0)) AppLcdUi_EnterPage();
  assert(actions==count);test_touch_range_valid=1;test_touch_count=0;AppLcdUi_PollTouch();
  /* A drive fault shows its name in red; a stop is spelled out in orange. */
  dm_motors[3].state=0xA;mock_safety.stop_reason=INFRABOT_STOP_DRIVE_FAULT;mock_status=ROBOT_STATUS_STOPPING;frame();
  assert(screen_has("OVERCURRENT") && slot_color("OVERCURRENT")==S_RED);
  assert(screen_has("STOP DRIVE_FAULT") && slot_color("STOP DRIVE_FAULT")==S_ORANGE);
  assert(small_ui.health_drawn==SH_DRIVE);
  shot(dir,"lcd43-arm-fault");
  dm_motors[3].state=1;mock_safety=(InfrabotSafetyStatus){0};mock_status=ROBOT_STATUS_IDLE_HOLD;frame();
  arm_waiting_mock=1;arm_responding_mock=3;frame();
  assert(screen_has("Waiting for arm power: 3/7") && small_ui.health_drawn==SH_NO_ARM);
  arm_waiting_mock=0;frame();assert(!screen_has("Waiting"));
  /* A shorter status clears the old tail. */
  assert(!screen_has("motors respond"));
  tap(SA_BACK,0);assert(small_ui.page==SP_MENU);
}

static void test_gravity(const char *dir) {
  open_page(SP_GRAVITY);
  assert(find(SA_FREE,0) && find(SA_LOCK,0) && find(SA_VIEW,0) && find(SA_VIEW,1));
  assert(screen_has("Damping Kd") && screen_has("Lock Kp") && screen_has("H7C bare v1"));
  assert(!strcmp(find(SA_FREE,0)->label,"FREE: ACTIVE") && !strcmp(find(SA_LOCK,0)->label,"LOCK"));
  assert(find(SA_FREE,0)->bg==S_GO && find(SA_LOCK,0)->bg==S_START);   /* FREE confirmed; LOCK can be started */
  shot(dir,"lcd43-gravity");
  unsigned before=drawing;frame();assert(drawing==before);
  tap(SA_FREE,0);assert(last_action==APP_LOCAL_FREE);
  tap(SA_LOCK,0);assert(last_action==APP_LOCAL_LOCK);
  /* Slider drives the selected row; capture blocks buttons until release. */
  const AppLcdSlider sl=SmallUi_ListSlider(.05f);
  small_touch(1,(uint16_t)(sl.x+sl.width),(uint16_t)(sl.y+20));assert(values[0]==maxs[0]);
  small_touch(1,(uint16_t)(S_STOP_X+10),12);assert(last_action==APP_LOCAL_LOCK && values[0]!=mins[0]);
  small_touch(1,(uint16_t)sl.x,(uint16_t)(sl.y+20));assert(values[0]==mins[0]);
  small_touch(0,0,0);assert(ui_slider.index==APP_LCD_SLIDER_NONE);
  tap_xy(40,(uint16_t)(S_BODY_Y+6+S_LIST_ROW_H+5));assert(small_ui.param==1);
  small_touch(1,(uint16_t)(sl.x+sl.width),(uint16_t)(sl.y+20));small_touch(0,0,0);
  assert(values[1]==maxs[1] && values[0]==mins[0]);
  assert(screen_has("applied") && screen_has("0.50..30.00"));
  shot(dir,"lcd43-gravity-kp");
  values[0]=.8f;values[1]=8.f;
  view(1);
  last_gravity_fault=(AppGravityFault){.sequence=1,.check=APP_GRAVITY_CHECK_SPEED,.joint=2,.value=3.2f,.limit=3.14f};
  speed_limited_mask=(1U<<2)|(1U<<5);frame();
  assert(screen_has("SHA256") && screen_has("J3: SPEED") && screen_has("3.200 / limit 3.140 rad/s"));
  assert(screen_has("J3 J6") && screen_has("25/26 us"));
  shot(dir,"lcd43-gravity-details");
  /* No slider on DETAILS. */
  small_touch(1,(uint16_t)(sl.x+sl.width),(uint16_t)(sl.y+20));small_touch(0,0,0);assert(values[0]==.8f);
  last_gravity_fault=(AppGravityFault){0};speed_limited_mask=0;
  tap(SA_BACK,0);
}

/* Each mode page: its own parameters, ENTER confirmed by the controller, STOP. */
static void test_modes(const char *dir) {
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;
  open_page(SP_JOINT_IMP);
  assert(screen_has("Joint Kp") && screen_has("Joint Kd") && !screen_has("Linear K"));
  /* Idle ENTER is the start colour, unlike the neutral buttons. */
  assert(!strcmp(find(SA_ENTER,0)->label,"ENTER") && find(SA_ENTER,0)->bg==S_START && S_START!=S_BUTTON);
  tap(SA_ENTER,0);assert(last_action==APP_LOCAL_JOINT_IMP);
  mock_local_control.pending_actions=1U<<APP_LOCAL_JOINT_IMP;frame();
  assert(!strcmp(find(SA_ENTER,0)->label,"WAITING") && screen_has("waiting for the controller"));
  mock_local_control.pending_actions=0;mock_compliance_mode=DM_ETH_CONTROL_JOINT_IMPEDANCE;mock_entry_permille=200;frame();
  assert(!strcmp(find(SA_ENTER,0)->label,"RAMP") && screen_has("20%"));
  mock_entry_permille=1000;frame();
  assert(!strcmp(find(SA_ENTER,0)->label,"ACTIVE") && screen_has("equilibrium latched"));
  shot(dir,"lcd43-joint-imp");
  /* The menu tile shows the active mode. */
  tap(SA_BACK,0);assert(small_ui.tile_drawn[4]==ST_LIVE);
  /* Slider changes only the selected parameter, request first, applied ramps. */
  open_page(SP_CART_IMP);
  assert(screen_has("Linear K") && screen_has("Rotation D") && screen_has("Joint damping") && !screen_has("Joint Kp"));
  for(unsigned row=0;row<5;++row) {
    tap_xy(40,(uint16_t)(S_BODY_Y+6+row*S_LIST_ROW_H+5));assert(small_ui.param==row);
    SmallList l;assert(SmallUi_PageList(&l) && l.n==5);
    const AppLcdSlider sl=SmallUi_ListSlider(l.p[row].step);
    AppImpedanceParameters before,after;AppImpedance_GetParameters(&before);
    small_touch(1,(uint16_t)(sl.x+sl.width),(uint16_t)(sl.y+20));
    /* Captured: dragging up onto the top bar presses neither BACK nor STOP;
     * the value still follows x (here: the right end). */
    const unsigned count=actions;
    small_touch(1,(uint16_t)(sl.x+sl.width),10);assert(small_ui.page==SP_CART_IMP && actions==count);
    small_touch(1,(uint16_t)(S_STOP_X+10),10);assert(actions==count);
    small_touch(0,0,0);AppImpedance_GetParameters(&after);
    const uint8_t k=l.index[row];
    assert(after.requested[k]==AppImpedance_ParameterMax(k) && after.applied[k]==before.applied[k]);
    for(unsigned j=0;j<APP_IMP_PARAMETERS;++j) if(j!=k) assert(after.requested[j]==before.requested[j]);
    AppImpedance_SetParameter(k,before.requested[k]);
  }
  tap(SA_ENTER,0);assert(last_action==APP_LOCAL_CART_IMP);
  mock_local_control=(AppLocalControlStatus){.action=APP_LOCAL_CART_IMP,.result=6};frame();
  assert(screen_has("Rejected:") && slot_color("Rejected:")==S_ORANGE);
  /* A rejected CART request leaves the confirmed JOINT mode untouched. */
  assert(!strcmp(find(SA_ENTER,0)->label,"ENTER"));
  shot(dir,"lcd43-cart-imp-rejected");
  mock_local_control=(AppLocalControlStatus){0};mock_compliance_mode=DM_ETH_CONTROL_CARTESIAN_IMPEDANCE;frame();
  view(1);frame();assert(screen_has("No live CART IMP sample"));
  mock_imp_diagnostics=(ArmImpedanceDiagnostics){.valid=1,.target={.3f,-.2f,.45f},
    .measured={.3f,-.2f,.35f},.position_error={0,0,.1f},.force={0,0,8}};frame();
  assert(screen_has("450.0") && screen_has("350.0") && screen_has("100.0") && !screen_has("No live"));
  shot(dir,"lcd43-cart-imp-live");
  AppImpedanceParameters before_diag,after_diag;AppImpedance_GetParameters(&before_diag);
  tap_xy(200,200);AppImpedance_GetParameters(&after_diag);assert(!memcmp(&before_diag,&after_diag,sizeof(before_diag)));
  mock_imp_diagnostics.valid=0;tap(SA_STOP,0);assert(last_action==APP_LOCAL_STOP);

  open_page(SP_JOINT_ADM);
  assert(screen_has("Joint K") && screen_has("Joint mass") && screen_has("Damping zeta") && !screen_has("Linear K"));
  tap(SA_ENTER,0);assert(last_action==APP_LOCAL_JOINT_ADM);
  mock_compliance_mode=DM_ETH_CONTROL_JOINT_ADMITTANCE;mock_entry_permille=1000;frame();
  assert(!strcmp(find(SA_ENTER,0)->label,"ACTIVE") && screen_has("virtual reference"));
  shot(dir,"lcd43-joint-adm");
  view(1);
  assert(screen_has("Static ratio") && screen_has("Friction beta") && screen_has("Drive gain") && screen_has("shared"));
  assert(AppAdmittance_BiasLearning());tap(SA_BIAS,0);assert(!AppAdmittance_BiasLearning());
  assert(!strcmp(find(SA_BIAS,0)->label,"Bias learn: OFF"));
  tap(SA_BIAS,0);assert(AppAdmittance_BiasLearning());
  shot(dir,"lcd43-joint-adm-shared");
  view(2);assert(screen_has("F N"));
  tap(SA_BACK,0);

  mock_compliance_mode=DM_ETH_CONTROL_IDLE;
  open_page(SP_CART_ADM);
  assert(screen_has("Linear K") && screen_has("Virtual mass") && screen_has("Return speed") && !screen_has("Joint mass"));
  shot(dir,"lcd43-cart-adm");
  view(2);assert(screen_has("Elbow null K") && screen_has("Elbow null D"));
  shot(dir,"lcd43-cart-adm-elbow");
  /* The shared view edits the same value the joint page shows. */
  view(1);tap_xy(40,(uint16_t)(S_BODY_Y+6+2*S_LIST_ROW_H+5));
  SmallList l;assert(SmallUi_PageList(&l) && l.index[2]==APP_ADM_DRIVE_GAIN);
  const AppLcdSlider sl=SmallUi_ListSlider(l.p[2].step);
  small_touch(1,(uint16_t)sl.x,(uint16_t)(sl.y+20));small_touch(0,0,0);
  AppAdmittanceParameters p;AppAdmittance_GetParameters(&p);
  assert(p.requested[APP_ADM_DRIVE_GAIN]==AppAdmittance_ParameterMin(APP_ADM_DRIVE_GAIN));
  AppAdmittance_SetParameter(APP_ADM_DRIVE_GAIN,1.f);
  view(3);mock_compliance_mode=DM_ETH_CONTROL_CARTESIAN_ADMITTANCE;
  mock_admittance=(AppAdmittanceStatus){.mode=DM_ETH_CONTROL_CARTESIAN_ADMITTANCE,.force={1,2,3},.moment={.1f,.2f,.3f},
    .error={.04f,.02f,.01f},.external={.12f},.tracking={.002f}};frame();
  assert(screen_has("1.0, 2.0, 3.0") && screen_has("Max ext J1"));
  shot(dir,"lcd43-cart-adm-live");
  mock_admittance.frozen=1;frame();assert(screen_has("ADM FROZEN: overload") && slot_color("FROZEN")==S_ORANGE);
  mock_admittance.frozen=0;mock_admittance.returning=1;frame();assert(screen_has("ADM RETURN"));
  mock_admittance=(AppAdmittanceStatus){0};mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;
  tap(SA_BACK,0);
}

static void test_network(const char *dir) {
  mock_eth=(AppEthernetInfo){.address={{192,168,0,50},{255,255,255,0},{192,168,0,1}},.mac={2,0,0,0x12,0x34,0x56},
    .link_up=1,.from_flash=1,.client_valid=1,.client_ip={192,168,0,10},.client_port=5000};
  open_page(SP_NETWORK);
  assert(screen_has("192.168.0.50") && screen_has("SAVED") && screen_has("NEW IP"));
  shot(dir,"lcd43-network");
  tap_xy(80,(uint16_t)(S_FIELD_Y(1)+4));assert(ui_net.selected==1);
  tap(SA_KEY,12);assert(!ui_net.field[1][0]);                  /* CLR */
  tap(SA_KEY,0);assert(!strcmp(ui_net.field[1],"1"));          /* 1 */
  tap(SA_KEY,11);assert(!ui_net.field[1][0]);                  /* DEL */
  tap(SA_KEY,13);assert(!strcmp(ui_net.field[1],"255.255.255.0"));   /* REVERT */
  /* An invalid address is refused; a valid one still needs every drive disabled. */
  ui_net.selected=0;tap(SA_KEY,12);tap(SA_KEY,15);assert(screen_has("IP is not a valid") && slot_color("IP is not")==UI_RED);
  test_network_valid=1;tap(SA_KEY,13);tap(SA_KEY,15);assert(screen_has("Disable all motors first"));
  test_network_valid=0;
  snprintf(ui_net.field[0],sizeof(ui_net.field[0]),"192.168.100.200");
  view(1);
  assert(screen_has("MAC") && screen_has("02:00:00:12:34:56") && screen_has("PC 192.168.0.10:5000"));
  assert(screen_has("192.168.100.200") && screen_has("changing the IP drops the PC link"));
  shot(dir,"lcd43-network-status");
  /* STATUS is read only: the keypad and field rows are gone. */
  assert(!find(SA_KEY,0));const uint8_t selected=ui_net.selected;tap_xy(80,(uint16_t)(S_FIELD_Y(2)+4));
  assert(ui_net.selected==selected);
  view(0);assert(!strcmp(ui_net.field[0],"192.168.100.200"));   /* switching views keeps the edit */
  tap(SA_BACK,0);
}

static void test_3d(const char *dir) {
  open_page(SP_3D);
  const unsigned renders=model_renders;frame();assert(model_renders>renders);
  assert(screen_has("TCP mm") && screen_has("Drag to rotate"));
  shot(dir,"lcd43-3d");
  small_touch(1,100,120);assert(ui_touch.dragging);
  small_touch(0,0,0);assert(!ui_touch.dragging);
  unsigned count=actions;small_touch(1,100,120);small_touch(1,(uint16_t)(S_STOP_X+10),12);small_touch(0,0,0);
  assert(actions==count);                                       /* a drag never presses STOP */
  tap(SA_VIEW3D,APP_LCD3D_VIEW_TOP);
  ui_3d_us=3000;ui_3d_max_us=4000;
  ui_report=(UiFrameReport){.ui_hz10=300,.draw_hz10=299,.last_us=3100,.max_us=4500,
    .interval_min_us=33000,.interval_max_us=34000,.over=2};
  view(1);
  const unsigned paused=model_renders;mock_tick+=200;frame();assert(model_renders==paused);
  assert(screen_has("geo 1000  px 2000") && screen_has("clr 100  ras 1500") && screen_has("cln 50  cpy 350"));
  char area[24];snprintf(area,sizeof(area),"area %u%%",(unsigned)(39040U*100U/(S_3D_W*S_3D_H)));
  assert(screen_has(area) && screen_has("3000/4000") && screen_has("30.0  draw 29.9"));
  assert(screen_has("3100/4500") && screen_has("33000/34000") && screen_has("over 2  SRAM  tiles 4"));
  shot(dir,"lcd43-3d-render");
  small_touch(1,100,120);assert(!ui_touch.dragging);small_touch(0,0,0);
  view(0);mock_tick+=200;frame();assert(model_renders>paused);
  tap(SA_BACK,0);
}

static void test_diag_info(const char *dir) {
  mock_stat.can_tx_errors=8;mock_stat.can_error_events=77;mock_stat.can_tec=9;mock_stat.can_rec=3;mock_stat.command_invalid=2;
  open_page(SP_DIAG);
  assert(screen_has("1000 Hz  miss 0") && screen_has("period max 1000 us") && screen_has("txerr 8  ev 77"));
  assert(screen_has("TEC 9  REC 3") && screen_has("invalid 2") && screen_has("GT1158") && screen_has("i2c_err 0"));
  assert(slot_color("txerr 8")==S_RED && small_ui.health_drawn==SH_CAN_ERR);
  shot(dir,"lcd43-diagnostics");
  tap(SA_BACK,0);assert(small_ui.tile_drawn[8]==ST_WARN);
  mock_stat.can_tx_errors=0;mock_stat.command_invalid=0;
  open_page(SP_SETTINGS);
  assert(find(SA_LANGUAGE,APP_UI_LANGUAGE_EN) && find(SA_LANGUAGE,APP_UI_LANGUAGE_ZH));
  view(1);
  assert(screen_has(APP_FIRMWARE_VERSION) && screen_has("dong1505lin@gmail.com"));
  assert(screen_has("the language is saved in flash"));
  shot(dir,"lcd43-info");
  view(2);for(unsigned i=0;i<8;i++){dm_motors[i].id=i+1;dm_motors[i].master_id=i+17;}frame();
  assert(screen_has("DM8009P-1  ") && screen_has("08/18"));
  shot(dir,"lcd43-info-motors");
  view(3);dm_control_stats.task_started=0;frame();assert(screen_has("--"));
  dm_control_stats.task_started=1;
  for(unsigned i=0;i<8;i++){eth_udp_command.target_kp[i]=123.45f;eth_udp_command.target_kd[i]=1.25f;
    dm_posvel_kp_apr[i]=54;eth_udp_command.motor_mode[i]=DM_ETH_MODE_MIT;}
  eth_udp_command.motor_mode[1]=DM_ETH_MODE_POS_VEL;frame();
  assert(screen_has("123.45") && screen_has("POS/VEL") && screen_has("POS/CUR"));
  shot(dir,"lcd43-info-pd");
  const unsigned count=actions;tap_xy(200,160);assert(actions==count);
  tap(SA_BACK,0);
}

/* ---- Language ---------------------------------------------------------------- */
static uint8_t language_buttons_ok(void) {
  const SmallButton *en=find(SA_LANGUAGE,APP_UI_LANGUAGE_EN),*zh=find(SA_LANGUAGE,APP_UI_LANGUAGE_ZH);
  return en && zh && !strcmp(en->label,"English") && !strcmp(zh->label,"简体中文") &&
         en->bg==(small_ui.language==APP_UI_LANGUAGE_EN?S_SELECT:S_BUTTON) &&
         zh->bg==(small_ui.language==APP_UI_LANGUAGE_ZH?S_SELECT:S_BUTTON);
}
static void set_enabled(uint8_t on) {
  dm_motor_runtime.control_enabled=on;for(unsigned i=0;i<7;++i) dm_motor_runtime.enabled[i]=on;
}

/* Choice, flash save only with every drive disabled, load at start. */
static void test_language(const char *dir) {
  /* No saved record (or a bad one): English. */
  mock_language_valid=0;SmallUi_LoadLanguage();assert(small_ui.language==APP_UI_LANGUAGE_EN);
  mock_language_valid=1;mock_language=APP_UI_LANGUAGE_ZH;SmallUi_LoadLanguage();
  assert(small_ui.language==APP_UI_LANGUAGE_ZH && small_ui.language_saved==APP_UI_LANGUAGE_ZH);
  mock_language=APP_UI_LANGUAGES;SmallUi_LoadLanguage();assert(small_ui.language==APP_UI_LANGUAGE_EN);
  mock_language_valid=0;SmallUi_LoadLanguage();

  open_page(SP_SETTINGS);
  assert(language_buttons_ok() && screen_has("Language of this 4.3in screen") && screen_has("Kept after reboot"));
  shot(dir,"lcd43-settings");
  /* Arm enabled: switches at once, not saved, says why. */
  const unsigned saves=language_saves;
  tap(SA_LANGUAGE,APP_UI_LANGUAGE_ZH);
  assert(small_ui.page==SP_SETTINGS && small_ui.view==0 && small_ui.language==APP_UI_LANGUAGE_ZH && language_saves==saves);
  assert(language_buttons_ok() && screen_has("未保存") && screen_has("失能所有电机后再点一次即可保存"));
  assert(screen_has("4.3 寸屏幕的显示语言") && slot_color("未保存")==S_ORANGE);
  assert(!strcmp(find(SA_STOP,0)->label,"停止"));
  shot(dir,"lcd43-settings-zh-unsaved");
  /* All drives disabled: the same tap saves. */
  set_enabled(0);tap(SA_LANGUAGE,APP_UI_LANGUAGE_ZH);
  assert(language_saves==saves+1 && mock_language==APP_UI_LANGUAGE_ZH && small_ui.language_saved==APP_UI_LANGUAGE_ZH);
  assert(screen_has("已保存到 MCU Flash") && slot_color("已保存到")==S_GREEN && !screen_has("未保存"));
  shot(dir,"lcd43-settings-zh");
  /* Tapping the saved language again writes nothing. */
  tap(SA_LANGUAGE,APP_UI_LANGUAGE_ZH);assert(language_saves==saves+1);
  /* Busy flash and a failed save are reported; nothing pretends to be saved. */
  mock_flash_busy=1;tap(SA_LANGUAGE,APP_UI_LANGUAGE_EN);
  assert(language_saves==saves+1 && small_ui.language==APP_UI_LANGUAGE_EN && screen_has("Flash busy"));
  mock_flash_busy=0;mock_save_ok=0;tap(SA_LANGUAGE,APP_UI_LANGUAGE_EN);
  assert(language_saves==saves+2 && small_ui.language_saved==APP_UI_LANGUAGE_ZH && screen_has("Save failed"));
  assert(slot_color("Save failed")==S_RED && screen_has("Not saved"));
  mock_save_ok=1;tap(SA_LANGUAGE,APP_UI_LANGUAGE_EN);
  assert(language_saves==saves+3 && mock_language==APP_UI_LANGUAGE_EN && screen_has("Saved in MCU flash"));
  /* The next start shows the saved language. */
  mock_language=APP_UI_LANGUAGE_ZH;SmallUi_LoadLanguage();assert(small_ui.language==APP_UI_LANGUAGE_ZH);
  mock_language=APP_UI_LANGUAGE_EN;SmallUi_LoadLanguage();
  set_enabled(1);
  tap(SA_BACK,0);
}

/* Every page and view in Chinese: layout invariants, no text cut, and only
 * characters the font has (AppLcd_DrawWide asserts the glyph). */
static void test_chinese(const char *dir) {
  small_ui.language=APP_UI_LANGUAGE_ZH;
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;
  if(small_ui.page!=SP_MENU) tap(SA_BACK,0);
  AppLcdUi_EnterPage();frame();
  const uint32_t cut=small_text_cut;
  assert(screen_has("已使能 7/7") && screen_has("J1-J7 关节弹簧") && screen_has("固件 test-revision"));
  assert(!strcmp(ui_slots[SS_STATE].text,"空闲保持"));
  shot(dir,"lcd43-zh-menu");
  static const char *const names[SP_COUNT]={"menu","arm","3d","gravity","network","joint-imp","cart-imp",
    "joint-adm","cart-adm","diagnostics","settings"};
  for(unsigned i=0;i<10;++i) {
    const uint8_t page=small_menu_pages[i];
    open_page(page);
    const unsigned views=small_view_count[page]?small_view_count[page]:1U;
    for(unsigned v=0;v<views;++v) {
      if(v) view((uint8_t)v);
      frame();
      char name[64];
      if(v) snprintf(name,sizeof(name),"lcd43-zh-%s-%u",names[page],v);
      else snprintf(name,sizeof(name),"lcd43-zh-%s",names[page]);
      shot(dir,name);
      if(small_text_cut!=cut) { fprintf(stderr,"text cut on %s\n",name); assert(!"Chinese text cut"); }
    }
    tap(SA_BACK,0);
  }
  /* Mode button states, rejection, stops and faults in Chinese. */
  open_page(SP_JOINT_IMP);
  assert(!strcmp(find(SA_ENTER,0)->label,"开启") && find(SA_ENTER,0)->bg==S_START);
  mock_local_control.pending_actions=1U<<APP_LOCAL_JOINT_IMP;frame();
  assert(!strcmp(find(SA_ENTER,0)->label,"等待中") && find(SA_ENTER,0)->bg==S_WAIT && screen_has("请求已排队"));
  mock_local_control.pending_actions=0;mock_compliance_mode=DM_ETH_CONTROL_JOINT_IMPEDANCE;mock_entry_permille=1000;frame();
  assert(!strcmp(find(SA_ENTER,0)->label,"运行中") && find(SA_ENTER,0)->bg==S_GO);
  shot(dir,"lcd43-zh-joint-imp-active");
  tap(SA_BACK,0);
  mock_compliance_mode=DM_ETH_CONTROL_IDLE;
  open_page(SP_CART_IMP);
  mock_local_control=(AppLocalControlStatus){.action=APP_LOCAL_CART_IMP,.result=6};frame();
  assert(screen_has("被拒绝：") && slot_color("被拒绝")==S_ORANGE);
  mock_local_control=(AppLocalControlStatus){0};tap(SA_BACK,0);
  open_page(SP_GRAVITY);
  assert(!strcmp(find(SA_FREE,0)->label,"自由") && !strcmp(find(SA_LOCK,0)->label,"锁定"));
  mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;frame();
  assert(!strcmp(find(SA_FREE,0)->label,"自由：运行中") && screen_has("自由：手动拖动"));
  last_gravity_fault=(AppGravityFault){.sequence=1,.check=APP_GRAVITY_CHECK_SPEED,.joint=2,.value=3.2f,.limit=3.14f};
  view(1);assert(screen_has("上次停止 J3：超速"));
  last_gravity_fault=(AppGravityFault){0};
  tap(SA_BACK,0);
  open_page(SP_ARM);
  dm_motors[3].state=0xA;mock_safety.stop_reason=INFRABOT_STOP_DRIVE_FAULT;mock_status=ROBOT_STATUS_STOPPING;frame();
  assert(screen_has("过流") && slot_color("过流")==S_RED && screen_has("停止 驱动故障") && small_ui.health_drawn==SH_DRIVE);
  assert(!strcmp(ui_slots[SS_STATE].text,"停止中"));
  shot(dir,"lcd43-zh-arm-fault");
  dm_motors[3].state=1;mock_safety=(InfrabotSafetyStatus){0};mock_status=ROBOT_STATUS_IDLE_HOLD;
  arm_waiting_mock=1;arm_responding_mock=3;frame();assert(screen_has("等待机械臂上电：3/7"));
  arm_waiting_mock=0;frame();
  tap(SA_BACK,0);
  /* The address editor's messages. */
  open_page(SP_NETWORK);
  ui_net.selected=0;tap(SA_KEY,12);tap(SA_KEY,15);assert(screen_has("IP 不是有效的"));
  test_network_valid=1;tap(SA_KEY,13);tap(SA_KEY,15);assert(screen_has("请先失能所有电机"));
  test_network_valid=0;view(1);assert(screen_has("修改 IP 会断开 PC 连接"));
  shot(dir,"lcd43-zh-network-message");
  tap(SA_BACK,0);
  assert(small_text_cut==cut);
  small_ui.language=APP_UI_LANGUAGE_EN;
  mock_compliance_mode=DM_ETH_CONTROL_GRAVITY_FREE;
  AppLcdUi_EnterPage();frame();
}

/* The table: format strings keep their conversions in order, the two table
 * headers keep their width, every character has a glyph, keys are unique. */
static void conversions(const char *f,char *out) {
  for(;*f;++f) if(*f=='%') {
    ++f;
    while(*f && !strchr("csudxXlf%",*f)) ++f;   /* flags and width may differ */
    if(*f=='l') *out++=*f++;
    *out++=*f;
  }
  *out=0;
}
static void test_table(void) {
  for(unsigned i=0;i<SMALL_TR_COUNT;++i) {
    char a[16],b[16];conversions(small_tr[i].en,a);conversions(small_tr[i].zh,b);
    if(strcmp(a,b)) { fprintf(stderr,"conversions differ: %s\n",small_tr[i].en); assert(!"format mismatch"); }
    for(const char *t=small_tr[i].zh;*t;) {
      const uint16_t c=AppLcdZh_Next(&t);
      assert(c!='?' || strchr(small_tr[i].en,'?'));
      if(c>=0x80) assert(AppLcdZh_Glyph(c));
    }
    for(unsigned j=i+1;j<SMALL_TR_COUNT;++j) assert(strcmp(small_tr[i].en,small_tr[j].en));
  }
  static const char *const headers[2]={"J  Model        ID/MST  PMAX  VMAX  TMAX","J  Mode        Kp       Kd      KP_APR"};
  static const char *const tails[2]={"ID/MST","Kp"};
  small_ui.language=APP_UI_LANGUAGE_ZH;
  for(unsigned i=0;i<2;++i) {
    const char *zh=SmallUi_Tr(headers[i]);assert(zh!=headers[i]);
    /* The columns after the first word start where the English ones do. */
    const char *a=strstr(headers[i],tails[i]),*b=strstr(zh,tails[i]);
    assert(a && b && !strcmp(a,b));
    char head[64];snprintf(head,sizeof(head),"%.*s",(int)(b-zh),zh);
    assert(AppLcdZh_Columns(head)==(unsigned)(a-headers[i]));
  }
  /* Every page title, state and health word fits in Chinese as well. */
  for(unsigned p=1;p<SP_COUNT;++p) assert(S_TITLE_X+SmallUi_Width(SmallUi_Tr(small_titles[p]))+4U<=S_STATE_X);
  for(uint8_t st=0;st<32;++st) assert(SmallUi_Width(SmallUi_Tr(SmallUi_StateName(st)))<=10U*8U);
  for(uint8_t h=0;h<=SH_DRIVE;++h) assert(SmallUi_Width(SmallUi_Tr(SmallUi_HealthName(h)))<=8U*8U);
  /* Unknown strings pass through; English mode never translates. */
  assert(!strcmp(SmallUi_Tr("H7C bare v1"),"H7C bare v1"));
  small_ui.language=APP_UI_LANGUAGE_EN;
  assert(!strcmp(SmallUi_Tr("STOP"),"STOP"));
  /* UTF-8 decoding: malformed input never runs past the end. */
  const char *bad="\xE5\x81",*t=bad;
  assert(AppLcdZh_Next(&t)=='?' && AppLcdZh_Next(&t)=='?' && AppLcdZh_Next(&t)==0);
  assert(AppLcdZh_Columns("停止 OK")==7U);
  uint16_t pal[16];AppLcdZh_Palette(S_TEXT,S_BG,pal);assert(pal[0]==S_BG && pal[15]==S_TEXT);
}

int main(int argc,char **argv) {
  const char *dir=argc>1?argv[1]:NULL;
#ifdef FW_NETBOOT_APP
  check_update_prompt(dir);
#endif
  dm_motor_runtime.control_enabled=1;
  for(unsigned i=0;i<7;++i) {dm_motor_runtime.enabled[i]=1;dm_motors[i].feedback_count=1;
    dm_motors[i].temperature_mos=39;dm_motors[i].state=1;
    dm_motor_runtime.command_position[i]=dm_motors[i].position=.05f;}
  /* Static layout facts. */
  for(unsigned p=1;p<SP_COUNT;++p) assert(S_TITLE_X+strlen(small_titles[p])*8U<=S_STATE_X);
  assert(S_STATE_X+4U*8U+16U+16U+7U*8U+8U<=S_STOP_X);   /* widest chip + dot + widest health word */
  for(uint8_t st=0;st<32;++st) assert(strlen(SmallUi_StateName(st))<=9U);
  for(uint8_t h=0;h<=SH_DRIVE;++h) assert(strlen(SmallUi_HealthName(h))<=7U);
  test_menu(dir);
  test_arm(dir);
  test_gravity(dir);
  test_modes(dir);
  test_network(dir);
  test_3d(dir);
  test_diag_info(dir);
  test_table();
  test_language(dir);
  test_chinese(dir);
  puts("4.3-inch menu UI: top bar/STOP on every page, tiles, view switches, lists/sliders, drags, all pages, "
       "language save rules and every page in Chinese passed");
}
#endif
