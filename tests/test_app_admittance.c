/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#define _POSIX_C_SOURCE 200809L
#include <unistd.h>
#define AppEthernet_GetTimeUs fixture_time
#define AppFdcan_BusFault fixture_bus
#define GRAVITY_HANDOVER_ENTRY previous_handover_main
#include "test_gravity_handover.c"
#undef GRAVITY_HANDOVER_ENTRY
#undef AppEthernet_GetTimeUs
#undef AppFdcan_BusFault
#include "app_admittance.h"
static uint32_t extra_us,time_step;
static uint8_t test_bus_fault;
uint64_t AppEthernet_GetTimeUs(void){extra_us+=time_step;return fixture_time()+extra_us;}
uint8_t AppFdcan_BusFault(void){return test_bus_fault;}
/* Exercise the same snapshot/clear and formatted line the operator sees. */
static void check_limiter_print_window(void){
  float q[7]={.2f,.9f,-.5f,1.3f,.3f,.6f,-.2f},dq[7]={0},gravity[7]={0},drive[7]={0};
  float reference[7],velocity[7],kp[7],kd[7],ff[7];
  float previous=0,peak_friction=0;
  ArmIkPose pose;ArmIk_Forward(q,&pose);
  AppAdmittance_Start(DM_ETH_CONTROL_JOINT_ADMITTANCE,q);
  drive[6]=-3.f;
  for(int i=0;i<15;i++){
    /* Deliberate gravity steps on J1 must only appear in total torque. */
    gravity[0]=drive[0]=(i%2)*.5f;
    assert(AppAdmittance_Evaluate(q,dq,gravity,drive,&pose,.001f,1,1000*i,
                                  reference,velocity,kp,kd,ff)==ARM_ADM_OK);
    AppAdmittance_RecordOutput(ff,drive,dq);
    float friction=.7f*(ArmGravity_Coulomb()[6]*tanhf(velocity[6]/ARM_ADM_FEEDFORWARD_WIDTH_RAD_S)+
                        ArmGravity_Viscous()[6]*velocity[6]);
    if(i)peak_friction=fmaxf(peak_friction,fabsf(friction-previous));
    previous=friction;
  }
  fflush(stdout);
  FILE *capture=tmpfile();assert(capture);
  int saved=dup(STDOUT_FILENO);assert(saved>=0);
  assert(dup2(fileno(capture),STDOUT_FILENO)>=0);
  AppAdmittance_Print();
  /* Enter the following window at zero reference velocity. The boundary
   * step must be included exactly once, then the next window has none. */
  for(int i=0;i<2;i++){
    assert(AppAdmittance_Evaluate(q,dq,gravity,drive,&pose,.001f,0,16000+i*1000,
                                  reference,velocity,kp,kd,ff)==ARM_ADM_OK);
    AppAdmittance_RecordOutput(ff,drive,dq);
    AppAdmittance_Print();
  }
  fflush(stdout);assert(dup2(saved,STDOUT_FILENO)>=0);close(saved);
  rewind(capture);
  char line[2048];
  assert(fgets(line,sizeof(line),capture));
  const char *fields=strstr(line," accel_bind_ms=");assert(fields);
  unsigned ticks,joint,vel_ticks;float peak;
  assert(sscanf(fields," accel_bind_ms=%u accel_need=%f accel_joint=%u vel_bind_ms=%u",
                &ticks,&peak,&joint,&vel_ticks)==4);
  assert(ticks>0 && ticks<=15 && peak>8 && joint==7 && !vel_ticks);
  assert(fgets(line,sizeof(line),capture));
  assert(strstr(line,"ADM JITTER n=15 dtau_max_mNm=500,"));
  char field[100];
  assert(lroundf(peak_friction*1000)>0 && fabsf(previous)>peak_friction);
  snprintf(field,sizeof(field)," ff_step_max_mNm=0,0,0,0,0,0,%ld ",lroundf(peak_friction*1000));
  assert(strstr(line,field));
  assert(fgets(line,sizeof(line),capture));
  assert(strstr(line," accel_bind_ms=0 accel_need=0.0 accel_joint=0 vel_bind_ms=0"));
  assert(fgets(line,sizeof(line),capture));
  snprintf(field,sizeof(field)," ff_step_max_mNm=0,0,0,0,0,0,%ld ",lroundf(fabsf(previous)*1000));
  assert(strstr(line,field));
  assert(fgets(line,sizeof(line),capture)); /* ADM */
  assert(fgets(line,sizeof(line),capture));
  assert(strstr(line," ff_step_max_mNm=0,0,0,0,0,0,0 "));
  fclose(capture);AppAdmittance_Stop();
}
int main(void){
  check_limiter_print_window();
  (void)previous_handover_main;
  for(unsigned mode=DM_ETH_CONTROL_JOINT_ADMITTANCE;mode<=DM_ETH_CONTROL_CARTESIAN_ADMITTANCE;mode++){
    setup(.5f);float before[7];for(int i=0;i<7;i++)before[i]=output(i);
    float gripper=dm_motor_runtime.command_position[7];
    assert(AppControlMode_Switch((uint8_t)mode)==0);
    for(int n=0;n<1100;n++){
      step();
      for(int i=0;i<7;i++)assert(fabsf(output(i)-before[i])<.0002f);
    }
    assert(dm_motor_runtime.command_position[7]==gripper && eth_udp_command.target_torque[7]==.3f);
    for(int i=0;i<7;i++)assert(eth_udp_command.target_kp[i]>0 && eth_udp_command.target_kd[i]>0);
    AppAdmittanceStatus s;AppAdmittance_Status(&s);assert(s.mode==mode);
    for(int i=0;i<7;i++)assert(fabsf(s.external[i])<1e-6f);
    /* A known signed external torque, no PC stream. */
    dm_motors[0].torque-=DM_MOTOR_DIRECTION(0)*1.5f;
    for(int n=0;n<100;n++)step();
    AppAdmittance_Status(&s);assert(s.mode==mode && s.external[0]>.2f);
    float load[7];for(int i=0;i<7;i++)load[i]=output(i);
    AppSafety_Stop(INFRABOT_STOP_REQUESTED,INFRABOT_NO_JOINT,0);
    for(int i=0;i<7;i++)assert(fabsf(output(i)-load[i])<.0002f);
    assert(AppGravity_Recovering() && dm_motor_runtime.control_enabled);
    for(int n=0;n<600;n++)step();
    assert(!AppGravity_Owned() && dm_motor_runtime.control_enabled);
    setup(1);assert(AppControlMode_Switch((uint8_t)mode)==0);
    tick+=60;AppGravity_Tick();assert(AppGravity_Recovering() && dm_motor_runtime.control_enabled);
    for(int n=0;n<600;n++)step();
    setup(1);assert(AppControlMode_Switch((uint8_t)mode)==0);
    for(int n=0;n<1100;n++)step();
    dm_motor_runtime.control_enabled=0;AppGravity_Tick();
    AppAdmittance_Status(&s);assert(!s.mode && !AppGravity_Owned());
    for(int i=0;i<7;i++)assert(eth_udp_command.target_velocity[i]==0);
  }
  /* J2 has negative drive direction. External torque and reference must
   * remain positive in the URDF joint frame and negative on the CAN side. */
  setup(1);assert(DM_MOTOR_DIRECTION(1)<0);
  assert(AppControlMode_Switch(DM_ETH_CONTROL_JOINT_ADMITTANCE)==0);
  for(int n=0;n<1100;n++)step();
  float j2_before=dm_motor_runtime.command_position[1];
  dm_motors[1].torque-=DM_MOTOR_DIRECTION(1)*1.5f;
  for(int n=0;n<60;n++)step();
  AppAdmittanceStatus signed_status;AppAdmittance_Status(&signed_status);
  assert(signed_status.external[1]>.2f && signed_status.velocity[1]>0);
  assert(dm_motor_runtime.command_position[1]<j2_before && eth_udp_command.target_velocity[1]>0);
  setup(1);assert(AppControlMode_Switch(DM_ETH_CONTROL_JOINT_ADMITTANCE)==0);
  for(int n=0;n<1100;n++)step();
  dm_motors[0].position-=DM_MOTOR_DIRECTION(0)*.1f;
  for(int n=0;n<550 && eth_udp_command.control_mode!=DM_ETH_CONTROL_IDLE;n++)step();
  InfrabotSafetyStatus safety={0};AppSafety_Report(&safety);AppAdmittanceStatus adm;AppAdmittance_Status(&adm);
  assert(safety.stop_reason==INFRABOT_STOP_INVALID_TARGET && safety.stop_detail==APP_GRAVITY_CHECK_MODEL && safety.stop_joint==0);
  assert(adm.fault==ARM_ADM_TRACKING && dm_motor_runtime.control_enabled);
  AppAdmittance_Print();
  setup(1);dm_motors[1].position=dm_motor_runtime.zero_offset[1]; /* at lower limit */
  assert(AppControlMode_Switch(DM_ETH_CONTROL_CARTESIAN_ADMITTANCE)==INFRABOT_REPLY_NOT_READY);
  /* Each inherited read_state/budget guard must also stop both new modes. */
  const uint8_t expected[]={APP_GRAVITY_CHECK_TEMPERATURE,APP_GRAVITY_CHECK_SPEED,
    APP_GRAVITY_CHECK_NONFINITE,APP_GRAVITY_CHECK_FEEDBACK,APP_GRAVITY_CHECK_CAN,
    APP_GRAVITY_CHECK_DRIVE,APP_GRAVITY_CHECK_BUDGET};
  for(unsigned mode=9;mode<=10;mode++)for(unsigned which=0;which<7;which++){
    setup(1);assert(AppControlMode_Switch((uint8_t)mode)==0);
    for(int n=0;n<1100;n++)step();
    if(which==0)dm_motors[0].temperature_coil=DM_GRAVITY_MAX_TEMP_C;
    if(which==1)dm_motors[0].velocity=5;
    if(which==2)dm_motors[0].torque=NAN;
    if(which==3)dm_motors[0].feedback_count=0;
    if(which==4)test_bus_fault=1;
    if(which==5)dm_motors[0].state=8;
    if(which==6)time_step=DM_GRAVITY_BUDGET_US+1;
    step();time_step=0;test_bus_fault=0;
    AppGravityStatus protected;AppGravity_Status(&protected);
    assert(eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE && dm_motor_runtime.control_enabled);
    assert(protected.recovering && protected.fault.check==expected[which]);
  }
  puts("ADM adapters: continuous total effort, sign, inherited temperature/speed/nonfinite/stale/CAN/drive/budget, disable, J8 and entry margin");
}
