/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#define GRAVITY_HANDOVER_ENTRY previous_handover_main
#include "test_gravity_handover.c"
#undef GRAVITY_HANDOVER_ENTRY
#include "arm_impedance.h"
#include "app_impedance.h"
int main(void) {
  (void)previous_handover_main;
  for(unsigned mode=DM_ETH_CONTROL_JOINT_IMPEDANCE;mode<=DM_ETH_CONTROL_CARTESIAN_IMPEDANCE;mode++) {
    for(unsigned share=0;share<3;share++) {
      setup(share*.5f);check_handover(mode); /* test total MIT output through ramp */
      for(unsigned i=0;i<7;i++)assert(eth_udp_command.target_kp[i]==0 && eth_udp_command.target_kd[i]==0);
      float eq[7],q[7],dq[7]={0},g[7],correction[7];ArmIkPose ep;
      for(unsigned i=0;i<7;i++)eq[i]=joint_q(i);
      ArmIk_Forward(eq,&ep);
      dm_motors[1].position+=DM_MOTOR_DIRECTION(1)*.01f;
      for(unsigned i=0;i<7;i++)q[i]=joint_q(i);
      assert(ArmGravity_Evaluate(q,dq,0,g,NULL));
      const ArmImpedanceGains gains={DM_GRAVITY_LOCK_KP,DM_GRAVITY_KD,
        DM_IMPEDANCE_TRANSLATION_K,DM_IMPEDANCE_TRANSLATION_D,
        DM_IMPEDANCE_ROTATION_K,DM_IMPEDANCE_ROTATION_D,DM_IMPEDANCE_JOINT_DAMPING};
      assert(ArmImpedance_Compute(mode==DM_ETH_CONTROL_CARTESIAN_IMPEDANCE,q,dq,eq,&ep,&gains,correction));
      for(unsigned n=0;n<1200;n++)step();
      for(unsigned i=0;i<7;i++) {
        assert(fabsf(output(i)-g[i]-correction[i])<1e-4f);
        assert(fabsf((dm_motor_runtime.command_position[i]-dm_motor_runtime.zero_offset[i])*DM_MOTOR_DIRECTION(i)-eq[i])<1e-5f);
      }
      /* A sudden displacement cannot jump the feedforward torque; the
       * independently added protective brake is zero at this zero velocity. */
      float old_tau[7];for(unsigned i=0;i<7;i++)old_tau[i]=eth_udp_command.target_torque[i];
      dm_motors[1].position+=DM_MOTOR_DIRECTION(1)*.15f;
      step();
      static const float cap[7]=DM_ARM_TORQUE_LIMIT_NM;
      for(unsigned i=0;i<7;i++) {
        assert(fabsf(eth_udp_command.target_torque[i]-old_tau[i])<=.85f*cap[i]/DM_CONTROL_HZ/DM_IMPEDANCE_TORQUE_RAMP_S+1e-5f);
        assert(fabsf(eth_udp_command.target_torque[i])<=.85f*cap[i]+1e-5f);
      }
      dm_motors[1].torque=-1.25f;step();AppGravityStatus status;AppGravity_Status(&status);
      assert(status.impedance.valid==(mode==DM_ETH_CONTROL_CARTESIAN_IMPEDANCE));
      if(status.impedance.valid) {
        ArmIkPose actual;float qs[7];for(unsigned i=0;i<7;i++)qs[i]=joint_q(i);ArmIk_Forward(qs,&actual);
        for(unsigned a=0;a<3;a++) {
          assert(fabsf(status.impedance.target[a]-ep.position[a])<1e-6f);
          assert(fabsf(status.impedance.measured[a]-actual.position[a])<1e-6f);
        }
      }
      assert(status.feedback_torque[1]==1.25f);
      AppSafety_Stop(INFRABOT_STOP_REQUESTED,INFRABOT_NO_JOINT,0);
      assert(AppGravity_Recovering());AppGravity_Status(&status);assert(!status.impedance.valid);
      for(unsigned n=0;n<600;n++)step();
      assert(dm_motor_runtime.control_enabled && !AppGravity_Owned());
      for(unsigned i=0;i<7;i++)assert(eth_udp_command.target_kp[i]>0 && dm_motor_runtime.enabled[i]);
    }
    /* Live impedance tuning is independent of gravity FREE/LOCK gains and
     * changes the torque law without re-latching its equilibrium. */
    setup(1);check_handover(mode);
    float eq_live[7],q_live[7],zero_live[7]={0},load_live[7],corr_live[7];ArmIkPose pose_live;
    for(unsigned i=0;i<7;i++)eq_live[i]=joint_q(i);
    ArmIk_Forward(eq_live,&pose_live);
    dm_motors[1].position+=DM_MOTOR_DIRECTION(1)*.01f;
    for(unsigned i=0;i<7;i++)q_live[i]=joint_q(i);
    AppImpedanceParameters old_params;AppImpedance_GetParameters(&old_params);
    const unsigned index=mode==DM_ETH_CONTROL_JOINT_IMPEDANCE?APP_IMP_JOINT_K:APP_IMP_TRANSLATION_K;
    assert(AppImpedance_SetParameter(index,old_params.requested[index]*2.f));
    assert(AppGravity_SetParameter(APP_GRAVITY_STIFFNESS,30.f));
    assert(AppGravity_SetParameter(APP_GRAVITY_DAMPING,3.f));
    for(unsigned n=0;n<2000;n++)step();
    AppImpedanceParameters new_params;AppImpedance_GetParameters(&new_params);
    assert(new_params.applied[index]==old_params.requested[index]*2.f);
    ArmImpedanceGains live={new_params.applied[0],new_params.applied[1],new_params.applied[2],
      new_params.applied[3],new_params.applied[4],new_params.applied[5],new_params.applied[6]};
    assert(ArmGravity_Evaluate(q_live,zero_live,0,load_live,NULL));
    assert(ArmImpedance_Compute(mode==DM_ETH_CONTROL_CARTESIAN_IMPEDANCE,q_live,zero_live,eq_live,&pose_live,&live,corr_live));
    for(unsigned i=0;i<7;i++) {
      assert(fabsf(output(i)-load_live[i]-corr_live[i])<1e-4f);
      assert(fabsf((dm_motor_runtime.command_position[i]-dm_motor_runtime.zero_offset[i])*DM_MOTOR_DIRECTION(i)-eq_live[i])<1e-5f);
    }
    assert(AppImpedance_SetParameter(index,old_params.requested[index]));
    assert(AppGravity_SetParameter(APP_GRAVITY_STIFFNESS,DM_GRAVITY_LOCK_KP));
    assert(AppGravity_SetParameter(APP_GRAVITY_DAMPING,DM_GRAVITY_KD));
    for(unsigned n=0;n<2000;n++)step();
    setup(1);assert(AppControlMode_Switch(mode)==0);
    tick+=60;AppGravity_Tick(); /* no refreshed feedback */
    assert(eth_udp_command.control_mode==DM_ETH_CONTROL_IDLE && AppGravity_Recovering());
    assert(dm_motor_runtime.control_enabled);
    setup(1);dm_motor_runtime.zero_valid=0;
    assert(AppControlMode_Switch(mode)==INFRABOT_REPLY_NOT_READY);dm_motor_runtime.zero_valid=1;
    setup(1);eth_udp_command.motor_mode[0]=DM_ETH_MODE_POS_VEL;
    assert(AppControlMode_Switch(mode)!=0);
  }
  puts("impedance adapters: joint/cartesian torque handover, fixed equilibrium, feedback direction, STOP hold, stale feedback, zero/profile gates passed");
}
