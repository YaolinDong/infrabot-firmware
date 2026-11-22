/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Native layouts for the independent impedance parameter groups. */
static const uint8_t imp_group[4][2]={
  {APP_IMP_JOINT_K,APP_IMP_JOINT_D},
  {APP_IMP_TRANSLATION_K,APP_IMP_TRANSLATION_D},
  {APP_IMP_ROTATION_K,APP_IMP_ROTATION_D},
  {APP_IMP_JOINT_DAMPING,APP_IMP_PARAMETERS}
};
static const char *const imp_names[APP_IMP_PARAMETERS]={
  "Joint Kp","Joint Kd","Linear K","Linear D","Rotation K","Rotation D","Joint damping"
};
static const char *const imp_units[APP_IMP_PARAMETERS]={
  "Nm/rad","Nm s/rad","N/m","N s/m","Nm/rad","Nm s/rad","Nm s/rad"
};
static const float imp_steps[APP_IMP_PARAMETERS]={.5f,.05f,5.f,1.f,.1f,.05f,.05f};
static unsigned ImpedanceSliders(AppLcdSlider sliders[2])
{
  const unsigned group=ui_detail[UI_PAGE_IMPEDANCE];
  if(group>=4U)return 0U;
  const unsigned count=group==3U?1U:2U;
  for(unsigned i=0;i<count;i++)
    sliders[i]=(AppLcdSlider){40U,(uint16_t)(202U+i*82U),480U,60U,imp_steps[imp_group[group][i]]};
  return count;
}
static void AppLcdUi_ImpedanceTouch(void)
{
  AppLcdSlider sliders[2];const unsigned count=ImpedanceSliders(sliders);
  AppLcdSlider_Capture(&ui_imp_slider,sliders,(uint8_t)count,ui_touch.x,ui_touch.y);
  if(ui_imp_slider.index>=count)return;
  const unsigned i=ui_imp_slider.index;
  const uint8_t parameter=imp_group[ui_detail[UI_PAGE_IMPEDANCE]][i];
  AppImpedance_SetParameter(parameter,AppLcdSlider_Value(&sliders[i],ui_touch.x,
    AppImpedance_ParameterMin(parameter),AppImpedance_ParameterMax(parameter)));
}
static void ImpedanceVector(char *line,size_t size,const char *label,const float v[3],float scale)
{
  char a[16],b[16],c[16];
  AppLcdUi_Fixed(a,sizeof(a),v[0]*scale,1);
  AppLcdUi_Fixed(b,sizeof(b),v[1]*scale,1);
  AppLcdUi_Fixed(c,sizeof(c),v[2]*scale,1);
  snprintf(line,size,"%s %s, %s, %s",label,a,b,c);
}
static void DrawImpedanceDiagnostics(const AppGravityStatus *state)
{
  const ArmImpedanceDiagnostics *d=&state->impedance;
  const uint8_t live=state->mode==DM_ETH_CONTROL_CARTESIAN_IMPEDANCE && d->valid;
  char line[100];
  for(unsigned row=0;row<8;row++) {
    line[0]=0;
    if(!live) {if(row==0) snprintf(line,sizeof(line),"No live CART IMP sample; enter CART IMP first.");}
    else switch(row) {
      case 0: ImpedanceVector(line,sizeof(line),"Target mm",d->target,1000.f);break;
      case 1: ImpedanceVector(line,sizeof(line),"Actual mm",d->measured,1000.f);break;
      case 2: ImpedanceVector(line,sizeof(line),"Error  mm",d->position_error,1000.f);break;
      case 3: ImpedanceVector(line,sizeof(line),"Rot err deg",d->rotation_error,57.2957795f);break;
      case 4: ImpedanceVector(line,sizeof(line),"Virtual F N",d->force,1.f);break;
      case 5: ImpedanceVector(line,sizeof(line),"Virtual M Nm",d->moment,1.f);break;
      case 6: snprintf(line,sizeof(line),"cap=%02X slew=%02X errcap=%02X brake=%02X",state->torque_clipped,
        state->torque_slew_limited,d->error_clamped,state->speed_limited_mask);break;
      case 7: {
        unsigned joint=0;for(unsigned i=1;i<7;i++)if(fabsf(state->impedance_torque[i])>fabsf(state->impedance_torque[joint]))joint=i;
        char value[16];AppLcdUi_Fixed(value,sizeof(value),state->impedance_torque[joint],3);
        snprintf(line,sizeof(line),"Largest correction J%u: %s Nm",joint+1U,value);break;
      }
    }
    AppLcdUi_Text(UI_SLOT_GRAV0+row,16,(uint16_t)(202+row*27),line,16,
      row==2?UI_HEADER:row==6?UI_YELLOW:UI_TEXT);
  }
  AppLcdUi_Text(UI_SLOT_DETAIL0+10,16,438,"Actual = FK feedback | virtual wrench != contact force",16,UI_DIM);
}
static void AppLcdUi_DrawImpedancePage(void)
{
  AppGravityStatus state;AppGravity_Status(&state);
  AppLocalControlStatus control;AppLocalControl_Status(&control);
  AppImpedanceParameters params;AppImpedance_GetParameters(&params);
  const unsigned group=ui_detail[UI_PAGE_IMPEDANCE];
  static const char *const groups[]={"JOINT: shared gains for J1-J7","CART: XYZ translation gains",
    "CART: XYZ rotation gains","CART: additional joint damping","CART: return diagnostics"};
  char line[112];
  AppLcdUi_Text(UI_SLOT_DETAIL0+2,16,62,groups[group],24,UI_HEADER);
  for(unsigned i=0;i<UI_BUTTON_COUNT;i++) {
    const uint8_t action=ui_buttons[i].action;
    if(action==UI_ACT_GRAV_JOINT_IMP || action==UI_ACT_GRAV_CART_IMP) {
      unsigned which=action==UI_ACT_GRAV_CART_IMP;
      if(ui_imp_indicator[which]!=ImpedanceIndicator(&state,&control,which))
        AppLcdUi_DrawButton(i,ui_pressed==i);
    }
  }
  const uint8_t joint=ImpedanceIndicator(&state,&control,0);
  const uint8_t cart=ImpedanceIndicator(&state,&control,1);
  const uint8_t active=joint>=2U || cart>=2U;
  uint16_t color=UI_DIM;
  if(state.recovering) {
    snprintf(line,sizeof(line),"STOPPING: restoring hold; impedance exited");color=UI_ORANGE;
  } else if(active) {
    const uint8_t entered=state.entry_permille>=1000U;
    snprintf(line,sizeof(line),"%s: %s IMPEDANCE%s",entered?"ACTIVE":"ENTERING",joint>=2U?"JOINT":"CARTESIAN",entered?"":" - ramping");
    color=entered?UI_GREEN:UI_YELLOW;
  } else if(joint==1U || cart==1U) {
    snprintf(line,sizeof(line),"WAITING: %s request",cart==1U?"CART IMP":"JOINT IMP");color=UI_YELLOW;
  } else {
    snprintf(line,sizeof(line),"NOT IN IMPEDANCE | %s",RobotController_StatusName(RobotController_OperationalStatus()));
  }
  AppLcdUi_Text(UI_SLOT_DETAIL0,16,154,line,16,color);
  color=UI_DIM;
  if(control.result && (control.action==APP_LOCAL_JOINT_IMP || control.action==APP_LOCAL_CART_IMP) &&
     state.mode!=(control.action==APP_LOCAL_JOINT_IMP?DM_ETH_CONTROL_JOINT_IMPEDANCE:DM_ETH_CONTROL_CARTESIAN_IMPEDANCE)) {
    const char *reason=control.result==4U?"wait for stop/setup; let arm settle":
      control.result==6U?"STOP first; verified MIT profile required":
      control.result==7U?"check enable, zero, feedback or faults":"check CAN/profile or request";
    snprintf(line,sizeof(line),"%s rejected: %s",control.action==APP_LOCAL_CART_IMP?"CART":"JOINT",reason);
    color=UI_ORANGE;
  } else if(active) {
    if(state.entry_permille>=1000U) snprintf(line,sizeof(line),"Entered; equilibrium latched. STOP holds enabled.");
    else snprintf(line,sizeof(line),"Entry ramp: %u%%; please wait",state.entry_permille/10U);
    color=state.entry_permille>=1000U?UI_GREEN:UI_YELLOW;
  } else if(joint==1U || cart==1U) {
    snprintf(line,sizeof(line),"Request queued; waiting for controller confirmation.");color=UI_YELLOW;
  } else if(state.fault.sequence) {
    snprintf(line,sizeof(line),"Last stop J%u: %s",state.fault.joint+1U,AppGravity_CheckName(state.fault.check));color=UI_ORANGE;
  } else snprintf(line,sizeof(line),"%s",AppLocalControl_Message(&control));
  AppLcdUi_Text(UI_SLOT_DETAIL0+1,16,176,line,16,color);
  if(group==4U) {DrawImpedanceDiagnostics(&state);return;}
  AppLcdSlider sliders[2];const unsigned count=ImpedanceSliders(sliders);
  for(unsigned i=0;i<count;i++) {
    const uint8_t p=imp_group[group][i];
    const AppLcdSlider *sl=&sliders[i];
    char requested[16],applied[16],lo[16],hi[16];
    AppLcdUi_Fixed(requested,sizeof(requested),params.requested[p],2);
    AppLcdUi_Fixed(applied,sizeof(applied),params.applied[p],2);
    AppLcdUi_Fixed(lo,sizeof(lo),AppImpedance_ParameterMin(p),2);
    AppLcdUi_Fixed(hi,sizeof(hi),AppImpedance_ParameterMax(p),2);
    snprintf(line,sizeof(line),"%s %s %s",imp_names[p],requested,imp_units[p]);
    AppLcdUi_Text(UI_SLOT_DETAIL0+4+2*i,sl->x,sl->y,line,16,UI_TEXT);
    snprintf(line,sizeof(line),"applied %s [%s..%s]",applied,lo,hi);
    AppLcdUi_Text(UI_SLOT_DETAIL0+5+2*i,sl->x,sl->y+17,line,16,UI_DIM);
    uint16_t knob=AppLcdSlider_Position(sl,params.requested[p],AppImpedance_ParameterMin(p),AppImpedance_ParameterMax(p));
    if(ui_imp_drawn[i]!=knob) {
      AppLcd_FillRect(sl->x-6,sl->y+35,sl->width+13,24,UI_BG);
      AppLcd_FillRect(sl->x,sl->y+43,sl->width,6,UI_LINE);
      AppLcd_FillRect(sl->x,sl->y+43,knob,6,UI_HEADER);
      AppLcd_FillRect(sl->x+knob-6,sl->y+35,13,22,UI_HEADER);
      ui_imp_drawn[i]=knob;
    }
  }
  if(group==3U) {
    AppLcdUi_Text(UI_SLOT_DETAIL0+6,40,294,"Acts in CART IMP on all joints.",16,UI_DIM);
    AppLcdUi_Text(UI_SLOT_DETAIL0+7,40,316,"No nullspace posture spring.",16,UI_DIM);
  }
  AppLcdUi_Text(UI_SLOT_LOCAL,568,202,"FF/Drive Nm",16,UI_HEADER);
  for(unsigned i=0;i<7;i++) {
    char tau[16],feedback[16];
    AppLcdUi_Fixed(tau,sizeof(tau),state.torque[i],2);
    AppLcdUi_Fixed(feedback,sizeof(feedback),state.feedback_torque[i],2);
    snprintf(line,sizeof(line),"J%u %s/%s",i+1,tau,feedback);
    AppLcdUi_Text(UI_SLOT_GRAV0+3+i,568,(uint16_t)(230+i*24),line,16,UI_TEXT);
  }
  snprintf(line,sizeof(line),"RAM only; reboot resets | calc %lu/%lu us",
           (unsigned long)state.last_us,(unsigned long)state.max_us);
  AppLcdUi_Text(UI_SLOT_DETAIL0+10,16,438,line,16,UI_YELLOW);
}
