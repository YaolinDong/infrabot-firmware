/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "control_services.h"
#include "platform.h"
#include "app_gravity.h"
#include "arm_gravity.h"
#include "arm_impedance.h"
#include "app_impedance.h"
#include "app_admittance.h"
#include "app_control_mode.h"
#include "app_state.h"
#include "app_safety.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

static AppGravityStatus stat;
static volatile uint8_t pending = 0xffU;
static const float parameter_min[] = {.1f, .5f}; /* Kd >= 0.1: never (almost) undamped */
static const float parameter_max[] = {3.f, 30.f};
static const float parameter_rate[] = {2.f, 20.f}; /* units/second */
static AppGravityParameters parameters = {
  {DM_GRAVITY_KD, DM_GRAVITY_LOCK_KP},
  {DM_GRAVITY_KD, DM_GRAVITY_LOCK_KP}
};
float AppGravity_ParameterMin(uint8_t i) { return i<APP_GRAVITY_PARAMETERS ? parameter_min[i] : 0.f; }
float AppGravity_ParameterMax(uint8_t i) { return i<APP_GRAVITY_PARAMETERS ? parameter_max[i] : 0.f; }
uint8_t AppGravity_SetParameter(uint8_t i,float value) {
  if(i>=APP_GRAVITY_PARAMETERS || !isfinite(value) || value<parameter_min[i] || value>parameter_max[i]) return 0U;
  platform_enter_critical(); parameters.value[i]=value; platform_exit_critical();
  return 1U;
}
void AppGravity_GetParameters(AppGravityParameters *out) {
  if(!out) return;
  platform_enter_critical(); *out=parameters; platform_exit_critical();
}
static float ramp;
static float start_q[7], start_kp[7], start_kd[7], hold_kp[7], hold_kd[7], stop_tau[7];
static float start_tau[7], start_v[7];
static float brake_magnitude[7], brake_velocity[7], stop_brake[7];
static float target[7], integral[7];
static ArmIkPose impedance_pose;
static float impedance_previous[7];
static const float torque_limit[7] = DM_ARM_TORQUE_LIMIT_NM;
static const float gravity_speed_limit[7] = DM_GRAVITY_MAX_SPEED_RAD_S;
static const float fallback_kp[7] = {300,300,150,150,40,40,30};
static const float fallback_kd[7] = {2.5f,2.5f,2.5f,2.5f,.8f,.8f,.8f};
static float clip(float v,float lo,float hi) { return fminf(fmaxf(v,lo),hi); }
static float smooth(float x) { x=clip(x,0.f,1.f); return x*x*(3.f-2.f*x); }
/* Always dissipative: added brake opposes the current measured velocity.
 * It cannot turn into a propulsive residual on release/reversal. */
static float speed_brake(unsigned i,float velocity,float dt) {
  const float speed=fabsf(velocity), limit=gravity_speed_limit[i];
  const float begin=DM_GRAVITY_SPEED_SOFT_RATIO*limit;
  if(speed<=begin || brake_velocity[i]*velocity<0.f) brake_magnitude[i]=0.f;
  brake_velocity[i]=velocity;
  const float maximum=DM_GRAVITY_SPEED_BRAKE_RATIO*torque_limit[i];
  const float wanted=maximum*smooth((speed-begin)/((DM_GRAVITY_SPEED_FULL_RATIO-
                                                DM_GRAVITY_SPEED_SOFT_RATIO)*limit));
  const float step=maximum*dt/DM_GRAVITY_SPEED_BRAKE_RAMP_S;
  brake_magnitude[i]+=clip(wanted-brake_magnitude[i],-step,step);
  if(speed>begin) stat.speed_limited_mask|=(uint8_t)(1U<<i);
  return velocity>=0.f ? -brake_magnitude[i] : brake_magnitude[i];
}
/* Dissipative only: brakes motion toward a joint limit that is too fast to
 * stop before it (allowed speed sqrt(2 a d)); zero when moving away. */
static float limit_brake(unsigned i,float q,float velocity) {
  if(velocity==0.f) return 0.f;
  const float distance=velocity>0.f ? ArmGravity_Upper()[i]-q : q-ArmGravity_Lower()[i];
  const float allowed=sqrtf(2.f*DM_GRAVITY_LIMIT_DECEL_RAD_S2*fmaxf(distance,0.f));
  const float excess=fabsf(velocity)-allowed;
  if(excess<=0.f) return 0.f;
  stat.speed_limited_mask|=(uint8_t)(1U<<i);
  const float brake=DM_GRAVITY_LIMIT_BRAKE_RATIO*torque_limit[i]*smooth(excess/DM_GRAVITY_LIMIT_BRAKE_SPAN);
  return velocity>0.f ? -brake : brake;
}
static float measured(unsigned i) { return (dm_motors[i].position-dm_motor_runtime.zero_offset[i])*DM_MOTOR_DIRECTION(i); }
static void set_position(unsigned i,float q) {
  dm_motor_runtime.command_position[i]=dm_motor_runtime.zero_offset[i]+DM_MOTOR_DIRECTION(i)*q;
  eth_udp_command.target_position[i]=q;
  dm_motor_runtime.override[i]=1U; dm_motor_runtime.stopped[i]=0U;
}
const char *AppGravity_CheckName(uint8_t check) {
  static const char *const names[]={"NONE","NONFINITE","SPEED","LOWER_LIMIT","UPPER_LIMIT",
    "MODEL_INVALID","CALC_BUDGET","CAN_FAULT","DRIVE_FAULT","STALE_FEEDBACK","TEMPERATURE"};
  return check<sizeof(names)/sizeof(names[0]) ? names[check] : "UNKNOWN";
}
static uint8_t check_failed(AppGravityFault *fault,uint8_t reason,uint8_t check,
                            unsigned joint,float value,float limit) {
  fault->check=check; fault->joint=(uint8_t)joint; fault->value=value; fault->limit=limit;
  return reason;
}
static uint8_t read_state(float q[7],float dq[7],AppGravityFault *fault) {
  const uint32_t now=platform_now_ms();
  if(AppFdcan_BusFault()) return check_failed(fault,INFRABOT_STOP_DRIVE_FAULT,
                                             APP_GRAVITY_CHECK_CAN,INFRABOT_NO_JOINT,0,0);
  uint8_t bad=0U;
  platform_enter_critical();
  for(unsigned i=0;i<7;++i) {
    q[i]=measured(i); dq[i]=dm_motors[i].velocity*DM_MOTOR_DIRECTION(i);
    if(!dm_motor_runtime.enabled[i] || eth_udp_command.motor_mode[i]!=DM_ETH_MODE_MIT || dm_motors[i].state!=1U) {
      bad=check_failed(fault,INFRABOT_STOP_DRIVE_FAULT,APP_GRAVITY_CHECK_DRIVE,i,dm_motors[i].state,1); break;
    }
    if(!dm_motors[i].feedback_count || (int32_t)(now-dm_motors[i].feedback_tick)>50) {
      bad=check_failed(fault,INFRABOT_STOP_FEEDBACK_LOST,APP_GRAVITY_CHECK_FEEDBACK,
                       i,(float)(int32_t)(now-dm_motors[i].feedback_tick),50); break;
    }
    if(!isfinite(q[i]) || !isfinite(dq[i]) || !isfinite(dm_motors[i].torque)) {
      bad=check_failed(fault,INFRABOT_STOP_INVALID_TARGET,APP_GRAVITY_CHECK_NONFINITE,i,0,0); break;
    }
    /* Normal limiting is torque feedback, not a mode exit. A separate
     * gross-overspeed boundary remains an immediate stop condition. */
    const float speed=fminf(gravity_speed_limit[i]*DM_GRAVITY_SPEED_TRIP_RATIO,
                            dm_motors[i].velocity_max);
    if(fabsf(dq[i])>speed) {
      bad=check_failed(fault,INFRABOT_STOP_INVALID_TARGET,APP_GRAVITY_CHECK_SPEED,i,dq[i],speed); break;
    }
    const float lower=ArmGravity_Lower()[i]-.05236f, upper=ArmGravity_Upper()[i]+.05236f;
    if(q[i]<lower) {
      bad=check_failed(fault,INFRABOT_STOP_INVALID_TARGET,APP_GRAVITY_CHECK_LOWER,i,q[i],lower); break;
    }
    if(q[i]>upper) {
      bad=check_failed(fault,INFRABOT_STOP_INVALID_TARGET,APP_GRAVITY_CHECK_UPPER,i,q[i],upper); break;
    }
    if(dm_motors[i].temperature_mos>=DM_GRAVITY_MAX_TEMP_C ||
       dm_motors[i].temperature_coil>=DM_GRAVITY_MAX_TEMP_C) {
      bad=check_failed(fault,INFRABOT_STOP_DRIVE_FAULT,APP_GRAVITY_CHECK_TEMPERATURE,i,
        fmaxf(dm_motors[i].temperature_mos,dm_motors[i].temperature_coil),DM_GRAVITY_MAX_TEMP_C); break;
    }
  }
  if(bad && fault->joint<7U) { fault->q=q[fault->joint]; fault->dq=dq[fault->joint]; }
  platform_exit_critical();
  return bad;
}
uint8_t AppGravity_CanStart(void) {
  float q[7],dq[7],tau[7];
  AppGravityFault fault={0};
  if(stat.recovering) return INFRABOT_REPLY_BUSY;
  if(!dm_motor_runtime.control_enabled || !dm_motor_runtime.zero_valid || read_state(q,dq,&fault)) return INFRABOT_REPLY_NOT_READY;
  /* The handover uses the existing command as well as feedback. Refuse a
   * corrupt hold before changing modes; never blend NaNs into CAN output. */
  for(unsigned i=0;i<7;++i)
    if(!isfinite(dm_motor_runtime.command_position[i]) || !isfinite(eth_udp_command.target_kp[i]) ||
       !isfinite(eth_udp_command.target_kd[i]) || !isfinite(eth_udp_command.target_torque[i]) ||
       !isfinite(eth_udp_command.target_velocity[i]) || eth_udp_command.target_kp[i]<0.f ||
       eth_udp_command.target_kp[i]>500.f || eth_udp_command.target_kd[i]<0.f || eth_udp_command.target_kd[i]>5.f)
      return INFRABOT_REPLY_INVALID;
  return ArmGravity_Evaluate(q,dq,0.f,tau,0) ? 0U : INFRABOT_REPLY_INVALID;
}
uint8_t AppGravity_CanStartMode(uint8_t mode){
  uint8_t result=AppGravity_CanStart();
  if(result || mode<DM_ETH_CONTROL_JOINT_ADMITTANCE)return result;
  float q[7];for(int i=0;i<7;i++)q[i]=measured(i);
  return AppAdmittance_EntryAllowed(q)?0U:INFRABOT_REPLY_NOT_READY;
}
void AppGravity_Start(uint8_t mode) {
  AppSafety_SetLocalHold(1U);
  stat.mode=mode; stat.recovering=0U; stat.entry_permille=0U; ramp=0;
  stat.speed_limited_mask=0U;
  stat.impedance.valid=stat.torque_clipped=stat.torque_slew_limited=0U;
  for(unsigned i=0;i<7;++i) {
    start_q[i]=(dm_motor_runtime.command_position[i]-dm_motor_runtime.zero_offset[i])*DM_MOTOR_DIRECTION(i);
    target[i]=measured(i); integral[i]=0;
    stat.feedback_torque[i]=dm_motors[i].torque*DM_MOTOR_DIRECTION(i);
    start_kp[i]=eth_udp_command.target_kp[i]; start_kd[i]=eth_udp_command.target_kd[i];
    /* Same effective velocity/torque as the CAN output, including a v4
     * per-drive STOP. Do not revive values masked out by dm_motor_runtime.stopped. */
    const float speed=fminf(dm_motors[i].velocity_max,DM_ARM_MAX_SPEED_RAD_S);
    start_v[i]=dm_motor_runtime.stopped[i] ? 0.f : clip(eth_udp_command.target_velocity[i],-speed,speed);
    start_tau[i]=dm_motor_runtime.stopped[i] ? 0.f : eth_udp_command.target_torque[i];
    impedance_previous[i]=start_tau[i];
    hold_kp[i]=start_kp[i]>=.5f ? start_kp[i] : fallback_kp[i];
    hold_kd[i]=start_kd[i]>=.1f ? start_kd[i] : fallback_kd[i];
    set_position(i,start_q[i]);
    eth_udp_command.target_velocity[i]=start_v[i];
    eth_udp_command.target_torque[i]=start_tau[i];
    stat.torque[i]=start_tau[i];
    brake_magnitude[i]=brake_velocity[i]=stat.speed_brake[i]=stop_brake[i]=0.f;
  }
  if(mode>=DM_ETH_CONTROL_JOINT_ADMITTANCE)AppAdmittance_Start(mode,target);
  ArmIk_Forward(target,&impedance_pose);
  eth_udp_command.active=0U; eth_udp_command.sequence_valid=0U;
  eth_udp_command.cartesian_target_valid=0U;
  eth_udp_command.queue_head=eth_udp_command.queue_tail=eth_udp_command.queue_count=0U;
  eth_udp_command.last_tick=platform_now_ms();
}
void AppGravity_OnStop(uint8_t reason) {
  const uint8_t adm=stat.mode>=DM_ETH_CONTROL_JOINT_ADMITTANCE;
  if(adm)AppAdmittance_Stop();
  if(!stat.mode) {
    if(stat.recovering && reason!=INFRABOT_STOP_REQUESTED && reason!=INFRABOT_STOP_STREAM_LOST)
      for(unsigned i=0;i<7;++i) stop_tau[i]=stop_brake[i]=eth_udp_command.target_torque[i]=0.f;
    return;
  }
  stat.impedance.valid=0U;
  stat.mode=0U; stat.recovering=1U; stat.entry_permille=0U; ramp=0;
  stat.speed_limited_mask=0U;
  for(unsigned i=0;i<7;++i) {
    const float q=measured(i);
    const float support=adm && isfinite(q)?
      eth_udp_command.target_kp[i]*((dm_motor_runtime.command_position[i]-dm_motor_runtime.zero_offset[i])*DM_MOTOR_DIRECTION(i)-q)+
      eth_udp_command.target_kd[i]*eth_udp_command.target_velocity[i]:0.f;
    if(isfinite(q)) set_position(i,q); /* Kp is restored at the actual stop pose */
    start_kp[i]=eth_udp_command.target_kp[i]; start_kd[i]=eth_udp_command.target_kd[i];
    stop_tau[i]=(reason==INFRABOT_STOP_REQUESTED || reason==INFRABOT_STOP_STREAM_LOST) ?
                  (adm?clip(eth_udp_command.target_torque[i]+support,-.85f*torque_limit[i],.85f*torque_limit[i]):eth_udp_command.target_torque[i]) : 0.f;
    stop_brake[i]=(reason==INFRABOT_STOP_REQUESTED || reason==INFRABOT_STOP_STREAM_LOST) ?
                    stat.speed_brake[i] : 0.f;
    stat.speed_brake[i]=brake_magnitude[i]=brake_velocity[i]=0.f;
    eth_udp_command.target_velocity[i]=0.f;
    eth_udp_command.target_torque[i]=stop_tau[i];
  }
}
uint8_t AppGravity_Owned(void) { return stat.mode || stat.recovering; }
uint8_t AppGravity_Recovering(void) { return stat.recovering; }
void AppGravity_Request(uint8_t mode) { pending=mode; }
void AppGravity_CancelRequest(void) { pending=0xffU; }
void AppGravity_ProcessRequest(void) {
  platform_enter_critical();
  const uint8_t mode=pending; pending=0xffU;
  platform_exit_critical();
  if(mode==0xffU) return;
  if(mode==DM_ETH_CONTROL_IDLE) {
    AppSafety_RequestStop(INFRABOT_STOP_REQUESTED); stat.request_status=0;
  } else stat.request_status=AppControlMode_Switch(mode);
}
void AppGravity_Status(AppGravityStatus *out) {
  if(!out) return;
  platform_enter_critical(); *out=stat; platform_exit_critical();
}
static void protection_stop(uint8_t reason,AppGravityFault *fault) {
  fault->sequence=stat.fault.sequence+1U;
  fault->kd=parameters.applied[APP_GRAVITY_DAMPING];
  stat.fault=*fault;
  const uint8_t detail=reason==INFRABOT_STOP_INVALID_TARGET ? fault->check :
      (fault->check==APP_GRAVITY_CHECK_DRIVE ? (uint8_t)fault->value : 0U);
  AppSafety_Stop(reason,fault->joint,detail);
}
/* newlib-nano does not provide float printf. INT32_MIN marks nonfinite or
 * unrepresentable diagnostic values; never cast NaN/Inf to an integer. */
static long diagnostic_milli(float value) {
  return isfinite(value) && fabsf(value)<2147483.f ? (long)lroundf(value*1000.f) : (long)INT32_MIN;
}
void AppGravity_PrintFault(void) {
  static uint32_t printed;
  AppGravityStatus snapshot; AppGravity_Status(&snapshot);
  const AppGravityFault *f=&snapshot.fault;
  if(f->sequence==printed) return;
  printed=f->sequence;
  const uint8_t milli=f->check==APP_GRAVITY_CHECK_SPEED || f->check==APP_GRAVITY_CHECK_LOWER ||
                      f->check==APP_GRAVITY_CHECK_UPPER;
  const char *unit=(f->check==APP_GRAVITY_CHECK_SPEED) ? "mrad/s" :
    (f->check==APP_GRAVITY_CHECK_LOWER || f->check==APP_GRAVITY_CHECK_UPPER) ? "mrad" :
    f->check==APP_GRAVITY_CHECK_BUDGET ? "us" :
    f->check==APP_GRAVITY_CHECK_FEEDBACK ? "ms" :
    f->check==APP_GRAVITY_CHECK_TEMPERATURE ? "C" : "raw";
  const long value=diagnostic_milli(f->value), limit=diagnostic_milli(f->limit);
  printf("GRAV_STOP seq=%lu check=%s joint=%u value=%ld limit=%ld unit=%s "
         "q_mrad=%ld dq_mrad_s=%ld kd_milli=%ld\r\n",
         (unsigned long)f->sequence,AppGravity_CheckName(f->check),
         f->joint<7U ? (unsigned)f->joint+1U : 0U,
         milli ? value : value/1000L,milli ? limit : limit/1000L,unit,
         diagnostic_milli(f->q),diagnostic_milli(f->dq),diagnostic_milli(f->kd));
}
void AppGravity_Tick(void) {
  const float dt=1.f/DM_CONTROL_HZ;
  ArmImpedanceGains impedance_gains;AppImpedance_Tick(dt,&impedance_gains);
  AppAdmittance_ParametersTick(dt);
  for(unsigned i=0;i<APP_GRAVITY_PARAMETERS;++i)
    parameters.applied[i]+=clip(parameters.value[i]-parameters.applied[i],-parameter_rate[i]*dt,parameter_rate[i]*dt);
  if(!AppGravity_Owned()) return;
  uint8_t all_enabled=dm_motor_runtime.control_enabled;
  for(unsigned i=0;i<7;++i) all_enabled &= dm_motor_runtime.enabled[i]!=0U;
  if(!all_enabled) {
    /* An explicit arm disable may leave J8 enabled. Restore normal gains
     * before a subsequent enable; never re-enable or move a drive here. */
    for(unsigned i=0;i<7;++i) {
      eth_udp_command.target_kp[i]=hold_kp[i]; eth_udp_command.target_kd[i]=hold_kd[i];
      eth_udp_command.target_torque[i]=0.f;
    }
    stat.impedance.valid=0U;
    if(stat.mode>=DM_ETH_CONTROL_JOINT_ADMITTANCE){AppAdmittance_Stop();for(int i=0;i<7;i++)eth_udp_command.target_velocity[i]=0;}
    stat.mode=stat.recovering=0U; stat.entry_permille=0U;
    stat.speed_limited_mask=0U;
    memset(stat.torque,0,sizeof(stat.torque));
    memset(stat.speed_brake,0,sizeof(stat.speed_brake));
    return;
  }
  if(stat.recovering) {
    ramp=fminf(ramp+dt/DM_GRAVITY_STOP_RAMP_S,1.f);
    for(unsigned i=0;i<7;++i) {
      if(!dm_motor_runtime.enabled[i]) continue;
      eth_udp_command.target_kp[i]=start_kp[i]+ramp*(hold_kp[i]-start_kp[i]);
      eth_udp_command.target_kd[i]=start_kd[i]+ramp*(hold_kd[i]-start_kd[i]);
      /* Do not retain a speed brake as a permanent holding bias. */
      eth_udp_command.target_torque[i]=stop_tau[i]-ramp*stop_brake[i];
      stat.torque[i]=eth_udp_command.target_torque[i];
    }
    if(ramp>=1.f) stat.recovering=0U;
    return;
  }
  float q[7],dq[7],tau[7],twist[6];
  AppGravityFault fault={.joint=INFRABOT_NO_JOINT};
  const uint8_t bad=read_state(q,dq,&fault);
  if(bad) { protection_stop(bad,&fault); return; }
  const uint64_t begin=platform_now_us();
  const uint8_t adm=stat.mode>=DM_ETH_CONTROL_JOINT_ADMITTANCE;
  float adm_q[7],adm_v[7],adm_kp[7],adm_kd[7],zero[7]={0};
  int ok=ArmGravity_Evaluate(q,adm?zero:dq,0.f,tau,twist); /* baseline g arithmetic unchanged */
  if(ok && adm){
    ArmIkPose actual;ArmIk_Forward(q,&actual);
    float drive[7];platform_enter_critical();
    for(int i=0;i<7;i++)drive[i]=dm_motors[i].torque*DM_MOTOR_DIRECTION(i);
    platform_exit_critical();
    ok=AppAdmittance_Evaluate(q,dq,tau,drive,&actual,dt,ramp>=1.f,begin,adm_q,adm_v,adm_kp,adm_kd,tau)==ARM_ADM_OK;
    if(!ok){fault.joint=AppAdmittance_FaultJoint();if(fault.joint<7){fault.q=q[fault.joint];fault.dq=dq[fault.joint];}}
  }
  const uint8_t impedance=stat.mode==DM_ETH_CONTROL_JOINT_IMPEDANCE ||
                          stat.mode==DM_ETH_CONTROL_CARTESIAN_IMPEDANCE;
  if(ok && impedance) {
    float correction[7];
    ok=ArmImpedance_Evaluate(stat.mode==DM_ETH_CONTROL_CARTESIAN_IMPEDANCE,q,dq,target,
                            &impedance_pose,&impedance_gains,correction,&stat.impedance);
    if(ok) {
      stat.impedance_gains=impedance_gains;
      for(unsigned i=0;i<7;i++){stat.gravity_torque[i]=tau[i];stat.impedance_torque[i]=correction[i];}
    }
    if(ok) for(unsigned i=0;i<7;i++) {tau[i]+=correction[i];if(!isfinite(tau[i]))ok=0;}
  }
  stat.last_us=(uint32_t)(platform_now_us()-begin);
  if(stat.last_us>stat.max_us) stat.max_us=stat.last_us;
  if(!ok || stat.last_us>DM_GRAVITY_BUDGET_US) {
    fault.check=ok ? APP_GRAVITY_CHECK_BUDGET : APP_GRAVITY_CHECK_MODEL;
    fault.value=ok ? (float)stat.last_us : 0.f;
    fault.limit=ok ? (float)DM_GRAVITY_BUDGET_US : 0.f;
    protection_stop(INFRABOT_STOP_INVALID_TARGET,&fault); return;
  }
  ramp=fminf(ramp+dt/DM_GRAVITY_RAMP_S,1.f);
  stat.entry_permille=(uint16_t)(ramp*1000.f);
  const float a=ramp*ramp*(3.f-2.f*ramp);
  const uint8_t lock=stat.mode==DM_ETH_CONTROL_GRAVITY_LOCK;
  const uint8_t moved=twist[0]*twist[0]+twist[1]*twist[1]+twist[2]*twist[2]>.04f*.04f ||
                      twist[3]*twist[3]+twist[4]*twist[4]+twist[5]*twist[5]>.08f*.08f;
  stat.speed_limited_mask=stat.torque_clipped=stat.torque_slew_limited=0U;
  for(unsigned i=0;i<7;++i) {
    stat.feedback_torque[i]=dm_motors[i].torque*DM_MOTOR_DIRECTION(i);
    if(!adm && !impedance && (!lock || moved)) { target[i]=q[i]; integral[i]*=.98951926f; }
    else if(!adm && !impedance && ramp>=1.f) integral[i]=clip(integral[i]+dt*(target[i]-q[i]),-.5f,.5f);
    set_position(i,(1.f-a)*start_q[i]+a*(adm?adm_q[i]:target[i]));
    const float next_kp=adm?adm_kp[i]:lock?parameters.applied[APP_GRAVITY_STIFFNESS]:0.f;
    const float next_kd=adm?adm_kd[i]:impedance?0.f:parameters.applied[APP_GRAVITY_DAMPING];
    eth_udp_command.target_velocity[i]=adm?(1.f-a)*start_v[i]+a*adm_v[i]:(1.f-a)*start_v[i];
    eth_udp_command.target_kp[i]=(1.f-a)*start_kp[i]+a*next_kp;
    eth_udp_command.target_kd[i]=(1.f-a)*start_kd[i]+a*next_kd;
    const float limit=.85f*torque_limit[i];
    const float requested_tau=tau[i]+(lock ? integral[i] : 0.f);
    if(fabsf(requested_tau)>limit)stat.torque_clipped|=(uint8_t)(1U<<i);
    const float next_tau=clip(requested_tau,-limit,limit);
    /* Blend the two complete MIT controllers, not gains and references
     * independently. Without this cross term, a PD-supported static load
     * loses 25% of support halfway through a FREE ramp: its Kp and position
     * error both shrink by (1-a), giving (1-a)^2 instead of (1-a).
     * The equivalent feedforward keeps q/v/gains continuous, including at
     * Kp=0, and vanishes at both endpoints. The existing torque cap remains. */
    const float cross=a*(1.f-a)*((start_kp[i]-next_kp)*(start_q[i]-(adm?adm_q[i]:target[i]))+
                                (start_kd[i]-next_kd)*(adm?start_v[i]-adm_v[i]:start_v[i]));
    float base=clip((1.f-a)*start_tau[i]+a*next_tau+cross,-limit,limit);
    if(impedance || adm) {
      const float step=limit*dt/DM_IMPEDANCE_TORQUE_RAMP_S;
      if(fabsf(base-impedance_previous[i])>step)stat.torque_slew_limited|=(uint8_t)(1U<<i);
      base=impedance_previous[i]+clip(base-impedance_previous[i],-step,step);
      impedance_previous[i]=base;
    }
    const float with_brake=base+speed_brake(i,dq[i],dt)+limit_brake(i,q[i],dq[i]);
    if(fabsf(with_brake)>limit)stat.torque_clipped|=(uint8_t)(1U<<i);
    stat.torque[i]=clip(with_brake,-limit,limit);
    stat.speed_brake[i]=stat.torque[i]-base;
    eth_udp_command.target_torque[i]=stat.torque[i];
  }
  if(adm)AppAdmittance_RecordOutput(stat.torque,stat.feedback_torque,dq);
  ++stat.ticks;
}

/* Reuse the exact control sample, without a second FK/Jacobian calculation.
 * Formatting runs at status-task priority, never in the 1 kHz task. */
void AppGravity_PrintImpedance(const AppGravityStatus *s)
{
  if(!s || s->mode!=DM_ETH_CONTROL_CARTESIAN_IMPEDANCE || !s->impedance.valid)return;
  const ArmImpedanceDiagnostics *d=&s->impedance;
  printf("IMP mode=%u ticks=%lu target_mm=%ld,%ld,%ld measured_mm=%ld,%ld,%ld "
         "err_mm=%ld,%ld,%ld rot_mrad=%ld,%ld,%ld cap=%02X slew=%02X errcap=%02X brake=%02X\r\n",
    s->mode,(unsigned long)s->ticks,diagnostic_milli(d->target[0]),diagnostic_milli(d->target[1]),diagnostic_milli(d->target[2]),
    diagnostic_milli(d->measured[0]),diagnostic_milli(d->measured[1]),diagnostic_milli(d->measured[2]),
    diagnostic_milli(d->position_error[0]),diagnostic_milli(d->position_error[1]),diagnostic_milli(d->position_error[2]),
    diagnostic_milli(d->rotation_error[0]),diagnostic_milli(d->rotation_error[1]),diagnostic_milli(d->rotation_error[2]),
    s->torque_clipped,s->torque_slew_limited,d->error_clamped,s->speed_limited_mask);
  printf("IMP gains_milli Kx/Dx=%ld/%ld Kr/Dr=%ld/%ld Dj=%ld\r\n",
    diagnostic_milli(s->impedance_gains.translation_k),diagnostic_milli(s->impedance_gains.translation_d),
    diagnostic_milli(s->impedance_gains.rotation_k),diagnostic_milli(s->impedance_gains.rotation_d),
    diagnostic_milli(s->impedance_gains.joint_damping));
  static const char *const labels[]={"model","imp","ff","drive"};
  const float *const arrays[]={s->gravity_torque,s->impedance_torque,s->torque,s->feedback_torque};
  for(unsigned a=0;a<4;a++) {
    printf("IMP %s_mNm=",labels[a]);
    for(unsigned i=0;i<7;i++)printf("%s%ld",i?",":"",diagnostic_milli(arrays[a][i]));
    printf("\r\n");
  }
}
