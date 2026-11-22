/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* ADM has independent RAM settings and confirmed controller-state feedback. */
static const uint8_t adm_group[6][2]={
 {APP_ADM_JOINT_K,APP_ADM_JOINT_MASS},{APP_ADM_LINEAR_K,APP_ADM_ROTATION_K},{APP_ADM_MASS,APP_ADM_ZETA},
 {APP_ADM_STATIC_RATIO,APP_ADM_FRICTION},{APP_ADM_DRIVE_GAIN,APP_ADM_RETURN_SPEED},{APP_ADM_NULL_K,APP_ADM_NULL_D}
};
static const char *const adm_names[]={"Joint K","Joint mass","Linear K","Rotation K","Mass","Zeta","Static ratio","Friction beta","Drive gain","Null K","Null D","Return speed"};
static const char *const adm_units[]={"Nm/rad","x","N/m","Nm/rad","kg","","x","","x","Nm/rad","Nm s/rad","mm/s"};
static const float adm_steps[]={1,.1f,10,.5f,.5f,.1f,.1f,.05f,.05f,.5f,.1f,10};
static unsigned AdmittanceSliders(AppLcdSlider sl[2]){
  unsigned group=ui_detail[UI_PAGE_ADMITTANCE];if(group>=6)return 0;
  for(int i=0;i<2;i++)sl[i]=(AppLcdSlider){40U,(uint16_t)(202U+i*82U),480U,60U,adm_steps[adm_group[group][i]]};
  return 2;
}
static void AppLcdUi_AdmittanceTouch(void){
  AppLcdSlider sl[2];unsigned n=AdmittanceSliders(sl);
  AppLcdSlider_Capture(&ui_adm_slider,sl,(uint8_t)n,ui_touch.x,ui_touch.y);
  if(ui_adm_slider.index>=n)return;
  unsigned i=ui_adm_slider.index,parameter=adm_group[ui_detail[UI_PAGE_ADMITTANCE]][i];
  AppAdmittance_SetParameter(parameter,AppLcdSlider_Value(&sl[i],ui_touch.x,AppAdmittance_ParameterMin(parameter),AppAdmittance_ParameterMax(parameter)));
}
static void AppLcdUi_DrawAdmittancePage(void){
  static AppAdmittanceStatus s;static AppGravityStatus g;
  AppAdmittance_Status(&s);AppGravity_Status(&g);
  AppLocalControlStatus c;AppLocalControl_Status(&c);AppAdmittanceParameters p;AppAdmittance_GetParameters(&p);
  unsigned group=ui_detail[UI_PAGE_ADMITTANCE];
  static const char *const groups[]={"JOINT: spring and virtual inertia","CART: virtual stiffness","CART: mass and damping ratio","Observer: deadzone and friction","Drive PD and return speed","CART: elbow mobility","ADM: sampled diagnostics"};
  char line[112];unsigned joint=AdmittanceIndicator(&g,&c,0),cart=AdmittanceIndicator(&g,&c,1);
  int active=joint>=2 || cart>=2;
  for(unsigned i=0;i<UI_BUTTON_COUNT;i++)if(ui_buttons[i].action==UI_ACT_ADM_JOINT || ui_buttons[i].action==UI_ACT_ADM_CART){
    unsigned which=ui_buttons[i].action==UI_ACT_ADM_CART;
    if(ui_adm_indicator[which]!=AdmittanceIndicator(&g,&c,which))AppLcdUi_DrawButton(i,ui_pressed==i);
  }
  uint16_t color=UI_DIM;
  if(g.recovering){snprintf(line,sizeof(line),"STOPPING: restoring hold");color=UI_ORANGE;}
  else if(active){
    if(g.entry_permille<1000){snprintf(line,sizeof(line),"ADM RAMP %u%%",g.entry_permille/10);color=UI_YELLOW;}
    else if(s.drift_frozen){snprintf(line,sizeof(line),"ADM FROZEN: IK lag");color=UI_ORANGE;}
    else if(s.frozen){snprintf(line,sizeof(line),"ADM FROZEN: overload");color=UI_ORANGE;}
    else if(s.returning){
      char value[16];float distance=sqrtf(s.error[0]*s.error[0]+s.error[1]*s.error[1]+s.error[2]*s.error[2]);
      AppLcdUi_Fixed(value,sizeof(value),distance*1000,1);snprintf(line,sizeof(line),"ADM RETURN %s mm",value);color=UI_GREEN;
    }else{snprintf(line,sizeof(line),"ADM ACTIVE: %s",cart>=2?"CARTESIAN":"JOINT");color=UI_GREEN;}
  }else if(joint==1 || cart==1){snprintf(line,sizeof(line),"WAITING: %s ADM",cart==1?"CART":"JOINT");color=UI_YELLOW;}
  else snprintf(line,sizeof(line),"NOT IN ADMITTANCE");
  AppLcdUi_Text(UI_SLOT_DETAIL0,16,154,line,16,color);
  if(c.result && (c.action==APP_LOCAL_JOINT_ADM || c.action==APP_LOCAL_CART_ADM) && !active){
    snprintf(line,sizeof(line),"Rejected: enable/zero/feedback; 0.03rad limit margin");color=UI_ORANGE;
  }else snprintf(line,sizeof(line),"%s",active?"Drive PD tracks virtual reference; STOP holds.":AppLocalControl_Message(&c));
  AppLcdUi_Text(UI_SLOT_DETAIL0+1,16,176,line,16,color);
  AppLcdUi_Text(UI_SLOT_DETAIL0+2,16,62,groups[group],24,UI_HEADER);
  if(group==6){
    for(int row=0;row<8;row++){
      line[0]=0;
      if(!active){if(row==0)snprintf(line,sizeof(line),"No live ADM sample.");}
      else switch(row){
        case 0:ImpedanceVector(line,sizeof(line),"F N",s.force,1);break;
        case 1:ImpedanceVector(line,sizeof(line),"M Nm",s.moment,1);break;
        case 2:ImpedanceVector(line,sizeof(line),"Feff N",s.effective_force,1);break;
        case 3:ImpedanceVector(line,sizeof(line),"e mm",s.error,1000);break;
        case 4:ImpedanceVector(line,sizeof(line),"rot deg",s.error+3,57.29578f);break;
        case 5:ImpedanceVector(line,sizeof(line),"vref mm/s",s.rate,1000);break;
        case 6:case 7:{
          const float *v=row==6?s.external:s.tracking;unsigned largest=0;
          for(unsigned j=1;j<7;j++)if(fabsf(v[j])>fabsf(v[largest]))largest=j;
          char value[16];AppLcdUi_Fixed(value,sizeof(value),v[largest]*(row==6?1:1000),row==6?3:1);
          snprintf(line,sizeof(line),"%s J%u %s %s",row==6?"Max ext":"Max track",largest+1,value,row==6?"Nm":"mrad");
        }break;
      }
      AppLcdUi_Text(UI_SLOT_GRAV0+row,16,(uint16_t)(202+row*27),line,16,UI_TEXT);
    }
  }else{
    AppLcdSlider sl[2];unsigned n=AdmittanceSliders(sl);
    for(unsigned i=0;i<n;i++){
      unsigned parameter=adm_group[group][i];char requested[16],value[16];
      AppLcdUi_Fixed(requested,sizeof(requested),p.requested[parameter],2);AppLcdUi_Fixed(value,sizeof(value),p.applied[parameter],2);
      snprintf(line,sizeof(line),"%s %s %s",adm_names[parameter],requested,adm_units[parameter]);
      AppLcdUi_Text(UI_SLOT_DETAIL0+4+2*i,sl[i].x,sl[i].y,line,16,UI_TEXT);
      snprintf(line,sizeof(line),"applied %s (RAM)",value);AppLcdUi_Text(UI_SLOT_DETAIL0+5+2*i,sl[i].x,sl[i].y+17,line,16,UI_DIM);
      uint16_t knob=AppLcdSlider_Position(&sl[i],p.requested[parameter],AppAdmittance_ParameterMin(parameter),AppAdmittance_ParameterMax(parameter));
      if(ui_adm_drawn[i]!=knob){
        AppLcd_FillRect(sl[i].x-6,sl[i].y+35,sl[i].width+13,24,UI_BG);
        AppLcd_FillRect(sl[i].x,sl[i].y+43,sl[i].width,6,UI_LINE);AppLcd_FillRect(sl[i].x,sl[i].y+43,knob,6,UI_HEADER);
        AppLcd_FillRect(sl[i].x+knob-6,sl[i].y+35,13,22,UI_HEADER);ui_adm_drawn[i]=knob;
      }
    }
    AppLcdUi_Text(UI_SLOT_LOCAL,568,202,"Ext/PD Nm",16,UI_HEADER);
    for(unsigned i=0;i<7;i++){
      char a[16],b[16];AppLcdUi_Fixed(a,sizeof(a),s.external[i],2);AppLcdUi_Fixed(b,sizeof(b),s.pd[i],2);
      snprintf(line,sizeof(line),"J%u %s/%s",i+1,a,b);
      AppLcdUi_Text(UI_SLOT_GRAV0+3+i,568,(uint16_t)(230+i*24),line,16,UI_TEXT);
    }
  }
  snprintf(line,sizeof(line),"RAM | %lu/%lu us",(unsigned long)g.last_us,(unsigned long)g.max_us);
  AppLcdUi_Text(UI_SLOT_DETAIL0+10,16,438,line,16,UI_YELLOW);
}
