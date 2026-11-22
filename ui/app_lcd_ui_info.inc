/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "control_stats.h"
#include "dm_runtime.h"
#include "eth_state.h"
/* Read-only information. Reads MCU state; no CAN requests or flash writes. */
static void InfoLine(unsigned row,uint16_t color,const char *format,...)
{
  char line[112];va_list args;va_start(args,format);
  vsnprintf(line,sizeof(line),format,args);va_end(args);
  AppLcdUi_Text((uint8_t)(UI_SLOT_DETAIL0+row),16U,(uint16_t)(60U+row*28U),line,24U,color);
}
static void AppLcdUi_DrawInfoPage(void)
{
  const unsigned page=ui_detail[UI_PAGE_INFO];
  if(page==0U) {
    InfoLine(0,UI_HEADER,"INFRABOT | Firmware & author");
    InfoLine(1,UI_TEXT,"MCU FW: %s",APP_FIRMWARE_VERSION);
    InfoLine(2,UI_DIM,"Build: %s %s",__DATE__,__TIME__);
    InfoLine(3,UI_TEXT,"Author: YaolinDong");
    InfoLine(4,UI_TEXT,"Email: dong1505lin@gmail.com");
    InfoLine(5,UI_TEXT,"MCU: STM32H743 | CPU %lu MHz",(unsigned long)(SystemCoreClock/1000000U));
    InfoLine(6,UI_TEXT,"LCD: %s %ux%u | UI %u Hz","5in",
      (unsigned)AppLcd_PhysicalWidth(),(unsigned)AppLcd_PhysicalHeight(),(unsigned)DM_LCD_FRAME_RATE_HZ);
    InfoLine(7,UI_TEXT,"Control: %u Hz | 8 motors",(unsigned)DM_CONTROL_HZ);
    InfoLine(8,UI_TEXT,"Profile: %s",dm_motor_runtime.profile_ready?"READY":"NOT READY");
    InfoLine(9,UI_DIM,"Firmware version = Git tag/revision (+dirty)");
    InfoLine(10,UI_DIM,"Read only | tap INFO for motor parameters");
    for(unsigned row=11U;row<14U;row++) InfoLine(row,UI_DIM,"");
  } else if(page==1U) {
    InfoLine(0,UI_HEADER,"MOTORS | MCU protocol encoding ranges");
    InfoLine(1,UI_DIM,"PMAX rad | VMAX rad/s | TMAX Nm");
    InfoLine(2,UI_TEXT,"J  Model        ID/MST  PMAX  VMAX  TMAX");
    for(unsigned i=0;i<DM_MOTOR_COUNT;i++) {
      char p[12],v[12],t[12];
      AppLcdUi_Fixed(p,sizeof(p),dm_motor_position_max[i],1U);
      AppLcdUi_Fixed(v,sizeof(v),dm_motor_velocity_max[i],1U);
      AppLcdUi_Fixed(t,sizeof(t),dm_motor_torque_max[i],1U);
      InfoLine(3U+i,UI_TEXT,"%u  %-11s %02X/%02X %5s %5s %5s",i+1U,dm_motor_names[i],
        (unsigned)dm_motors[i].id,(unsigned)dm_motors[i].master_id,p,v,t);
    }
    InfoLine(11,UI_DIM,"Ranges above are NOT software safety limits.");
    InfoLine(12,UI_DIM,"J8: position/current | tap INFO for PD");
    InfoLine(13,UI_DIM,"");
  } else {
    InfoLine(0,UI_HEADER,"PD | MCU MIT profile + drive KP_APR");
    InfoLine(1,UI_DIM,"Kp Nm/rad | Kd Nm s/rad | APR drive gain");
    InfoLine(2,UI_TEXT,"J  Mode        Kp       Kd      KP_APR");
    for(unsigned i=0;i<DM_MOTOR_COUNT;i++) {
      char kp[12]="--",kd[12]="--",apr[12]="--";
      if(dm_control_stats.task_started) {
        AppLcdUi_Fixed(kp,sizeof(kp),eth_udp_command.target_kp[i],2U);
        AppLcdUi_Fixed(kd,sizeof(kd),eth_udp_command.target_kd[i],2U);
        if(i<DM_ARM_MOTOR_COUNT) AppLcdUi_Fixed(apr,sizeof(apr),dm_posvel_kp_apr[i],2U);
      }
      const char *mode=i==DM_GRIPPER_INDEX?"POS/CUR":
                       (eth_udp_command.motor_mode[i]==DM_ETH_MODE_MIT?"MIT":eth_udp_command.motor_mode[i]==DM_ETH_MODE_POS_VEL?"POS/VEL":eth_udp_command.motor_mode[i]==DM_ETH_MODE_VEL?"VEL":"UNSET");
      InfoLine(3U+i,UI_TEXT,"%u  %-8s %8s %8s %8s",i+1U,mode,kp,kd,apr);
    }
    AppGravityParameters g;AppGravity_GetParameters(&g);
    char kp[12],kd[12];AppLcdUi_Fixed(kp,sizeof(kp),g.applied[APP_GRAVITY_STIFFNESS],2U);
    AppLcdUi_Fixed(kd,sizeof(kd),g.applied[APP_GRAVITY_DAMPING],2U);
    InfoLine(11,UI_TEXT,"Gravity applied: LOCK Kp %s | Kd %s",kp,kd);
    InfoLine(12,UI_DIM,"APR: cached/default; MIT Kp/Kd not POS gains.");
    InfoLine(13,UI_DIM,dm_control_stats.task_started?"Read only | tap INFO for version":"Waiting for motor startup; profile not loaded");
  }
}
