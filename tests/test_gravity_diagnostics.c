/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Real gravity and safety owners, synthetic CAN feedback only. No robot IO.
 * Low Kd is numerically legal. Feedback outside the physical bounds must
 * still stop, now with the actual offending joint. */
#define main app_motion_tests_main
#define AppEthernet_GetTimeUs Diagnostic_BaseTimeUs
#include "test_app_motion.c"
#undef main
#undef AppEthernet_GetTimeUs
#include "app_control_mode.h"
#include "app_gravity.h"
#include "arm_gravity.h"

static uint8_t bus_fault;
static uint8_t slow_model;
uint64_t AppEthernet_GetTimeUs(void) {
  static uint64_t elapsed;
  if(slow_model) elapsed+=300;
  return (uint64_t)tick*1000+elapsed;
}
uint8_t AppFdcan_BusFault(void) { return bus_fault; }
uint8_t AppTasks_Enabling(void) { return 0; }
static void step(void) {
  ++tick;
  for(unsigned i=0;i<7;++i) dm_motors[i].feedback_tick=tick;
  AppGravity_Tick(); AppSafety_Tick();
}
static void setup_mode(uint8_t mode) {
  dm_motor_runtime.control_enabled=0; AppGravity_Tick();
  ready(DM_ETH_MODE_MIT); eth_udp_command.control_mode=DM_ETH_CONTROL_IDLE;
  static const float encoding_speed[7]={45,45,10,10,30,30,30};
  for(unsigned i=0;i<7;++i) {
    dm_motors[i].velocity_max=encoding_speed[i];
    dm_motors[i].position=dm_motor_runtime.zero_offset[i]+DM_MOTOR_DIRECTION(i)*.1f;
    dm_motor_runtime.command_position[i]=dm_motors[i].position;
    eth_udp_command.target_kp[i]=50; eth_udp_command.target_kd[i]=2;
    eth_udp_command.target_velocity[i]=eth_udp_command.target_torque[i]=0;
  }
  assert(AppGravity_SetParameter(APP_GRAVITY_DAMPING,.1f));
  assert(AppControlMode_Switch(mode)==0);
  for(unsigned i=0;i<1500;++i) step();
  assert(eth_udp_command.control_mode==mode);
}
static void setup(void) { setup_mode(DM_ETH_CONTROL_GRAVITY_FREE); }
static void stopped(uint8_t check,unsigned joint) {
  AppGravityStatus g; AppGravity_Status(&g);
  assert(g.fault.check==check && g.fault.joint==joint);
  assert(g.fault.kd==.1f);
  assert(eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE && dm_motor_runtime.control_enabled && g.recovering);
  InfrabotSafetyStatus s={0}; AppSafety_Report(&s);
  assert(s.stop_joint==joint);
  if(s.stop_reason==INFRABOT_STOP_INVALID_TARGET) assert(s.stop_detail==check);
  const AppGravityFault captured=g.fault;
  for(unsigned i=0;i<600;++i) step();
  AppGravity_Status(&g);
  assert(!g.recovering && !g.mode);
  assert(g.fault.sequence==captured.sequence && g.fault.value==captured.value);
  assert(g.fault.limit==captured.limit); /* evidence survives the hold */
}
int main(void) {
  dm_motor_runtime.zero_valid=1;
  setup();
  AppGravityStatus g; AppGravity_Status(&g); const uint32_t sequence=g.fault.sequence;
  const float speeds[]={0.f,.15f,.30f,.5f,1.f,2.9f,3.14f,3.143f,3.187f,3.2f};
  for(unsigned n=0;n<sizeof(speeds)/sizeof(speeds[0]);++n)
    for(unsigned sign=0;sign<2;++sign) {
      for(unsigned i=0;i<7;++i) dm_motors[i].velocity=DM_MOTOR_DIRECTION(i)*speeds[n]*(sign ? -1.f : 1.f);
      for(unsigned t=0;t<20;++t) step();
      AppGravity_Status(&g);
      assert(g.mode==DM_ETH_CONTROL_GRAVITY_FREE && !g.recovering && g.fault.sequence==sequence);
      for(unsigned i=0;i<7;++i) assert(isfinite(g.torque[i]));
    }
  static const float drag_limit[7]=DM_GRAVITY_MAX_SPEED_RAD_S;
  for(unsigned mode=DM_ETH_CONTROL_GRAVITY_FREE;mode<=DM_ETH_CONTROL_GRAVITY_LOCK;++mode)
  for(unsigned i=0;i<7;++i) for(unsigned sign=0;sign<2;++sign) {
    setup_mode((uint8_t)mode);
    const float direction=sign ? -1.f : 1.f;
    /* Slightly above the 3.14 limit does not exit either local gravity mode. */
    dm_motors[i].velocity=DM_MOTOR_DIRECTION(i)*direction*3.187f;
    for(unsigned n=0;n<20;++n) step();
    AppGravity_Status(&g);
    assert(g.mode==mode && !g.recovering);
    /* The ordinary limit applies braking without leaving gravity mode. */
    dm_motors[i].velocity=DM_MOTOR_DIRECTION(i)*direction*drag_limit[i];
    step();
    AppGravity_Status(&g);
    assert(g.mode==mode && !g.recovering);
    dm_motors[i].velocity=DM_MOTOR_DIRECTION(i)*direction*drag_limit[i]*1.01f;
    for(unsigned n=0;n<100;++n) step();
    AppGravity_Status(&g);
    assert(g.mode==mode && !g.recovering && (g.speed_limited_mask & (1U<<i)));
    assert(g.speed_brake[i]*direction<0.f);
    const float trip=drag_limit[i]*DM_GRAVITY_SPEED_TRIP_RATIO;
    const float v=direction*nextafterf(trip,INFINITY);
    dm_motors[i].velocity=DM_MOTOR_DIRECTION(i)*v;
    step();
    AppGravity_Status(&g);
    assert(g.fault.value==v && g.fault.dq==v && g.fault.limit==trip);
    stopped(APP_GRAVITY_CHECK_SPEED,i);
  }
  AppGravity_PrintFault(); /* exercise the once-per-event post-IDLE logger */
  for(unsigned i=0;i<7;++i) for(unsigned upper=0;upper<2;++upper) {
    setup();
    const float q=upper ? ArmGravity_Upper()[i]+.06f : ArmGravity_Lower()[i]-.06f;
    dm_motors[i].position=dm_motor_runtime.zero_offset[i]+DM_MOTOR_DIRECTION(i)*q;
    step();
    stopped(upper ? APP_GRAVITY_CHECK_UPPER : APP_GRAVITY_CHECK_LOWER,i);
  }
  setup(); dm_motors[2].velocity=NAN; step(); stopped(APP_GRAVITY_CHECK_NONFINITE,2);
  AppGravity_PrintFault();
  setup(); dm_motors[4].temperature_coil=90; step(); stopped(APP_GRAVITY_CHECK_TEMPERATURE,4);
  setup(); slow_model=1; step(); stopped(APP_GRAVITY_CHECK_BUDGET,INFRABOT_NO_JOINT);
  AppGravity_Status(&g); assert(g.fault.value==300 && g.fault.limit==DM_GRAVITY_BUDGET_US);
  slow_model=0;
  setup(); bus_fault=1; step(); stopped(APP_GRAVITY_CHECK_CAN,INFRABOT_NO_JOINT);
  puts("gravity diagnostics: FREE/LOCK governor at 3.14 rad/s, gross-overspeed stop both directions, position limits, nonfinite, temperature, bus fault: OK");
}
