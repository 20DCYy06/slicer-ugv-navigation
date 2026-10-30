/* Actual firmware parser/outer loop/IMU conversion, with only MCU registers and
 * physical I2C replies replaced. No board access and no flashing. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifndef FIRMWARE_UNDER_TEST
#define FIRMWARE_UNDER_TEST "../main.c"
#endif
#define main firmware_main
#include FIRMWARE_UNDER_TEST
#undef main
static uint8_t bus_registers[256],bus_data[12];
static int bus_ok=1,bus_data_ready=1;
int test_imu_write(uint8_t reg,uint8_t value){bus_registers[reg]=value;return bus_ok;}
int test_imu_read(uint8_t reg,uint8_t*out,unsigned size){
 if(!bus_ok)return 0;
 if(reg==0&&size==1){*out=0xea;return 1;}
 if(reg==0x1a&&size==1){*out=bus_data_ready;return 1;}
 if(reg==0x2d&&size==12){memcpy(out,bus_data,12);return 1;}
 return 0;
}
static void raw(unsigned offset,int16_t value){bus_data[offset]=(uint16_t)value>>8;bus_data[offset+1]=value;}
static void feed(unsigned kind,const uint8_t*p,unsigned n,int corrupt){
 uint8_t b[73]={0xa5,0x5a,1,(uint8_t)kind,(uint8_t)n,1,0};
 for(unsigned i=0;i<n;i++)b[i+7]=p[i];
 p16(b+7+n,crc(b+2,n+5)^(corrupt?1:0));
 for(unsigned i=0;i<n+9;i++)parse(b[i]);
}
static void reset(void){
 stop();clear_fault();hello_ok=1;configured=1;nonce=123;ms=0;
 control_interval_ms=CONTROL_MS;battery_mv=12000;gd.IDR=8;ad.DR=1354;ad.SR=ADC_SR_EOC;u1.SR=USART_SR_TXE;
 const int8_t ps[4]=DEFAULT_PWM_SIGNS,es[4]=DEFAULT_ENCODER_SIGNS;
 for(unsigned i=0;i<4;i++){
  enc[i]->CNT=previous[i]=0;filtered[i]=0;actual[i]=0;
  pwm_sign[i]=ps[i];enc_sign[i]=es[i];
 }
 bus_ok=bus_data_ready=1;memset(bus_data,0,sizeof(bus_data));
 raw(4,16384);icm20948_init();
}
static void body_cmd(int vx,int wz,int corrupt){
 uint8_t p[8];p32(p,nonce);p16(p+4,vx);p16(p+6,wz);feed(BODY_COMMAND,p,8,corrupt);
}
static void manual_cmd(int value){
 uint8_t p[12];p32(p,nonce);for(unsigned i=0;i<4;i++)p16(p+4+2*i,value);feed(0x11,p,12,0);
}
static void imu_and_pid(void){
 reset();assert(imu.present&&imu.who==0xea&&!imu.calibrated);
 assert(bus_registers[1]==0x33&&bus_registers[0x14]==0x31);
 unsigned yaw_offset=6+2*IMU_YAW_AXIS;
 raw(yaw_offset,100);
 for(unsigned n=1;n<=IMU_BIAS_SAMPLES;n++)icm20948_sample(n*CONTROL_MS,actual,1);
 assert(imu.calibrated&&imu.bias_raw==100.f);
 raw(yaw_offset,1100);icm20948_sample((IMU_BIAS_SAMPLES+1)*CONTROL_MS,actual,1);
 const int gyro_signs[3]=IMU_GYRO_SIGNS;
 int converted=imu.gyro_mrad_s[IMU_YAW_AXIS]*gyro_signs[IMU_YAW_AXIS];
 assert(converted>260&&converted<270);
 assert(imu.accel_mm_s2[2]>9800&&imu.accel_mm_s2[2]<9815);
 assert(icm20948_ready(imu.sample_ms));
 assert(!icm20948_ready(imu.sample_ms+IMU_TIMEOUT_MS+1));
 int16_t wheels[4];body_reset();
 body_update(0,1,0,0,.01,wheels);
 assert(wheels[0]>0&&wheels[2]>0&&wheels[1]<0&&wheels[3]<0);
 float first=body.yaw_correction;
 body_update(0,1,0,.8,.01,wheels);assert(body.yaw_correction<first);
 body_reset();body_update(.2,0,0,0,.01,wheels);
 assert(wheels[0]>200&&wheels[0]==wheels[1]);
 body_update(0,0,0,.01,.01,wheels);
 assert(!body.yaw.integral&&!body.linear.integral&&!wheels[0]);
 for(unsigned n=0;n<2000;n++)body_update(.6,3.5,0,0,.01,wheels);
 for(unsigned i=0;i<4;i++)assert((unsigned)absi(wheels[i])<=MAX_MM_S);
 assert(body.yaw.integral<=BODY_W_CAP&&body.linear.integral<=BODY_V_CAP);
 bus_ok=0;icm20948_sample(imu.sample_ms+10,actual,1);assert(imu.failed);
 icm20948_init();assert(!icm20948_ready(0)&&!imu.initialized);
}
static void protocol_and_arbitration(void){
 reset();imu.calibrated=imu.have_sample=1;imu.sample_ms=0;
 body_cmd(200,1000,1);assert(!sources.nav_valid);
 uint8_t bad[8];p32(bad,nonce+1);p16(bad+4,200);p16(bad+6,1000);
 feed(BODY_COMMAND,bad,8,0);assert(!sources.nav_valid);
 body_cmd(MAX_BODY_MM_S+1,0,0);assert(!sources.nav_valid);
 body_cmd(200,1000,0);control();
 assert(active_source==SOURCE_BODY&&have_command&&targets[0]>targets[1]);
 manual_cmd(350);ms=10;control();
 assert(active_source==SOURCE_PWM&&duty[0]==560&&direct_mode);
 assert(t10.CCR1==1600&&t11.CCR1==1040);
 ms=20;body_cmd(100,0,0);control();assert(active_source==SOURCE_PWM);
 manual_cmd(0);ms=30;control();assert(active_source==SOURCE_BODY&&!direct_mode);
 ms=520;control();assert(!have_command&&active_source==SOURCE_STOP);
 reset();manual_cmd(350);control();ms=501;control();assert(!have_command&&!duty[0]);
 reset();imu.calibrated=imu.have_sample=1;body_cmd(200,0,0);control();
 bus_data_ready=0;ms=IMU_TIMEOUT_MS+1;control();assert(!have_command&&!fault&&!duty[0]);
 manual_cmd(350);control();assert(active_source==SOURCE_PWM&&duty[0]==560);
 uint8_t emergency[8]={0};feed(0x10,emergency,8,0);assert(!have_command&&!sources.nav_valid&&!sources.manual_valid);
 reset();imu.calibrated=imu.have_sample=1;body_cmd(200,0,0);control();
 gd.IDR=0;control();assert(!have_command&&!duty[0]);
}
static void bridge_and_saturation(void){
 reset();
 /* 四路闭环/开环前进方向及反向的实际CCR逐一相等；占空比取相同物理ticks。 */
 const int test_signs[4]={1,-1,1,-1};
 for(unsigned i=0;i<4;i++)for(int direction=-1;direction<=1;direction++){
  zero();pwm(i,direction*pwm_sign[i]*PWM_CAP);
  uint32_t closed[8]={t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4};
  zero();test_pwm(i,direction*test_signs[i]*PWM_CAP);
  uint32_t opened[8]={t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4};
  for(unsigned n=0;n<8;n++)assert(closed[n]==opened[n]);
 }
 reset();targets[0]=targets[2]=800;targets[1]=targets[3]=400;
 wheel_control(.01f);
 assert(absi(duty[0])==PWM_CAP&&absi(duty[2])==PWM_CAP);
 assert(absi(duty[1])==PWM_CAP/2&&absi(duty[3])==PWM_CAP/2);
 for(unsigned i=0;i<4;i++)assert(integral[i]==0);
 /* 速度积分不能把改变方向后的微小负目标卡在正方向。 */
 zero();integral[0]=700;previous_target[0]=200;targets[0]=-20;
 targets[1]=targets[2]=targets[3]=0;wheel_control(.01f);
 assert(integral[0]<0&&duty[0]*pwm_sign[0]<0);
 reset();body_update(.4,.5,.4,0,.01,targets);
 float positive=body.yaw_correction;
 body_reset();body_update(.4,.5,.4,.8,.01,targets);
 assert(body.yaw_correction<positive); /* 实际偏航过快时IMU反馈减小转向力。 */
}
static void telemetry_and_legacy(void){
 reset();ms=40;control();telemetry();
 assert(last_tx[3]==BODY_TELEMETRY&&last_tx[4]==64&&last_tx_size==73);
 assert(previous_tx[3]==0x90&&previous_tx[4]==61);
 assert(crc(last_tx+2,last_tx_size-4)==rd16(last_tx+last_tx_size-2));
 assert(last_tx[7]==1&&last_tx[9]==0xea);
 assert(rd16(last_tx+7+56)==battery_mv);
 uint8_t req[4];p32(req,nonce);feed(BODY_INFO_REQUEST,req,4,0);
 assert(last_tx[3]==BODY_INFO_REPLY&&last_tx[4]==8&&rd32(last_tx+7)==nonce);
 feed(0x13,req,4,0);assert(last_tx[3]==0x83&&last_tx[4]==9);
 uint8_t speed[8];for(unsigned n=0;n<4;n++)p16(speed+2*n,200);
 feed(0x10,speed,8,0);control();assert(active_source==SOURCE_WHEEL&&targets[0]==200);
 ms=540;control();assert(!have_command);
 reset();uint8_t cal[9];p32(cal,nonce);cal[4]=2;p16(cal+5,CAL_PERMILLE);p16(cal+7,CAL_MS);
 feed(0x20,cal,9,0);control();assert(active_source==SOURCE_CAL&&duty[2]>0);
 ms=CAL_MS;control();assert(!have_command&&!duty[2]);
}
/* The selected main.c and its actual parser/control/PI/bridge run unchanged.
 * Trace input is synthetic, not a reconstruction of unrecorded 100Hz feedback. */
static FILE *yaw_trace;
static unsigned yaw_failures;
static void yaw_record(const char *name,unsigned step,int command_w){
 if(!yaw_trace)return;
 fprintf(yaw_trace,"%s,%u,%u,%d,%.9g,%.9g,%.9g,%.9g,%u,%u,%u",
  name,step,(unsigned)ms,command_w,body.measured_yaw,body.yaw.integral,
  body.linear.integral,body.yaw_correction,(unsigned)fault,
  (unsigned)fault_source,(unsigned)active_source);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%d",targets[i]);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%.9g",integral[i]);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%d",duty[i]*pwm_sign[i]);
 for(unsigned i=0;i<4;i++)fprintf(yaw_trace,",%d",duty[i]);
 fprintf(yaw_trace,",%u,%u,%u,%u,%u,%u,%u,%u\n",
  t10.CCR1,t11.CCR1,t9.CCR1,t9.CCR2,t1.CCR1,t1.CCR2,t1.CCR3,t1.CCR4);
 for(unsigned i=0;i<4;i++){
  assert((unsigned)absi(targets[i])<=MAX_MM_S);
  assert((unsigned)absi(duty[i])<=PWM_CAP);
 }
}
static void yaw_ready(void){
 reset();body=(BodyControl){0};
 imu.calibrated=imu.have_sample=1;imu.sample_ms=0;
}
static void yaw_step(int vx,int wz){
 ms+=CONTROL_MS;control_interval_ms=CONTROL_MS;
 body_cmd(vx,wz,0);control();
}
static int yaw_direction_ok(int wz){
 for(unsigned i=0;i<4;i++){
  int side=(i==0||i==2)?1:-1;
  if((int32_t)targets[i]*wz*side<=0||
     (int32_t)(duty[i]*pwm_sign[i])*wz*side<=0)return 0;
 }
 return 1;
}
static void yaw_reverse(int first,int next){
 char name[64];snprintf(name,sizeof(name),"reverse_%d_%d",first,next);
 yaw_ready();
 for(unsigned n=0;n<200;n++){yaw_step(0,first);yaw_record(name,n,first);}
 assert(!fault&&body.yaw.integral*first>0);
 unsigned failures_before=yaw_failures;
 for(unsigned n=0;n<250;n++){
  yaw_step(0,next);yaw_record(name,n+200,next);
  if(n<10&&!yaw_direction_ok(next))yaw_failures++;
  if(fault)break; /* zero feedback may legitimately trip unchanged protection */
 }
 printf("ACCEPT %s first_100ms=%s\n",name,
        yaw_failures==failures_before?"PASS":"FAIL");
}
static void yaw_hold(int w){
 char name[64];snprintf(name,sizeof(name),"hold_%d",w);yaw_ready();
 for(unsigned n=0;n<400;n++){
  yaw_step(0,w);yaw_record(name,n,w);
  if(!n)assert(yaw_direction_ok(w));
  if(n==199)assert(body.yaw.integral*w>0);
  if(fault){assert(fault==4&&fault_source==5&&!have_command);break;}
 }
}
static void yaw_amplitude(int sign){
 char name[64];snprintf(name,sizeof(name),"same_amplitude_%d",sign);yaw_ready();
 for(unsigned n=0;n<300;n++){
  int w=sign*(n<100?52:(n<200?60:100));
  yaw_step(0,w);yaw_record(name,n,w);if(fault)break;
 }
}
static void yaw_periodic(int magnitude,unsigned period,int sign){
 char name[64];snprintf(name,sizeof(name),"alternate_%d_%u_%d",magnitude,period,sign);
 yaw_ready();
 for(unsigned n=0;n<800;n++){
  int w=((n/period)%2?-1:1)*sign*magnitude;
  yaw_step(0,w);yaw_record(name,n,w);if(fault)break;
 }
}
static void yaw_outer_step(float vx,float w,float measured_vx,float measured_w){
 ms+=CONTROL_MS;body_update(vx,w,measured_vx,measured_w,.01f,targets);
 for(unsigned i=0;i<4;i++)actual[i]=(int16_t)(filtered[i]=measured_vx*1000);
 wheel_control(.01f);
}
static void yaw_braking(int requested,int measured){
 char name[64];snprintf(name,sizeof(name),"brake_%d_%d",requested,measured);
 yaw_ready();
 for(unsigned n=0;n<30;n++){
  yaw_outer_step(0,requested*.001f,0,measured*.001f);
  yaw_record(name,n,requested);
  assert(measured>0?targets[0]<targets[1]:targets[0]>targets[1]);
 }
}
static void yaw_straight_and_arcs(void){
 yaw_ready();
 /* Natural linear-I history; no extreme integral is injected. */
 for(unsigned n=0;n<100;n++)yaw_outer_step(.2f,0,.1f,0);
 float linear_i=body.linear.integral;assert(linear_i>0);
 for(unsigned n=0;n<100;n++){
  yaw_outer_step(.2f,0,.2f,.02f);yaw_record("straight_w0",n,0);
  assert(targets[0]<targets[1]&&body.linear.integral==linear_i);
 }
 yaw_ready();
 for(unsigned n=0;n<400;n++){
  const int sequence[8]={1,0,-1,0,1,-1,0,0};int w=sequence[(n/10)%8];
  yaw_outer_step(.2f,w*.001f,.2f,.002f);yaw_record("micro_arc",n,w);
  assert(targets[0]>0&&targets[1]>0&&targets[2]>0&&targets[3]>0);
 }
 yaw_ready();
 for(unsigned n=0;n<300;n++){
  int w=n<100?60:(n<200?0:-60);
  yaw_outer_step(.2f,w*.001f,.2f,0);yaw_record("arc_cross_zero",n,w);
  for(unsigned i=0;i<4;i++)assert(targets[i]>0);
 }
 /* Micro pure turn: quantization is allowed, not promoted to motion. */
 yaw_ready();
 for(unsigned n=0;n<400;n++){
  int w=n<200?1:-1;yaw_step(0,w);yaw_record("micro_pure",n,w);
  for(unsigned i=0;i<4;i++)assert(!targets[i]&&!duty[i]);
  /* Reversal may reject old yaw I, but must not amplify a sub-resolution
   * request into motion or reset the unrelated PID history. */
  if(n==200)assert(body.yaw.started&&!body.linear.integral);
 }
}
static void yaw_reset_and_limits(void){
 yaw_ready();
 for(unsigned n=0;n<200;n++)yaw_step(0,60);
 yaw_step(0,-52);
 /* Subsequent wheel saturation must retain the corrected I, not old +I. */
 float corrected_i=body.yaw.integral;
 for(unsigned n=0;n<20;n++){
  body_update(.8f,-3.5f,0,0,.01f,targets);
  wheel_control(.01f);yaw_record("saturation",n,-3500);
  assert(body.yaw.integral==corrected_i);
 }
 yaw_ready();for(unsigned n=0;n<200;n++)yaw_step(0,60);
 yaw_step(0,0);yaw_record("all_zero",0,0);
 assert(!body.yaw.integral&&!body.linear.integral);
 for(unsigned i=0;i<4;i++)assert(!targets[i]&&!duty[i]&&!integral[i]);
 yaw_ready();for(unsigned n=0;n<100;n++)yaw_step(0,60);
 manual_cmd(350);ms+=CONTROL_MS;control();yaw_record("switch_pwm",0,0);
 assert(active_source==SOURCE_PWM&&!body.yaw.integral);
 manual_cmd(0);yaw_step(0,-60);yaw_record("switch_body",0,-60);
 assert(active_source==SOURCE_BODY&&yaw_direction_ok(-60));
 yaw_ready();yaw_step(0,60);bus_data_ready=0;
 for(unsigned n=0;n<=IMU_TIMEOUT_MS/CONTROL_MS;n++){
  yaw_step(0,60);yaw_record("imu_stale",n,60);
 }
 assert(!have_command&&!fault&&!body.yaw.integral);
 for(unsigned i=0;i<4;i++)assert(!duty[i]);
 yaw_ready();yaw_step(0,60);ms+=WATCHDOG_MS-1;control();
 yaw_record("watchdog_before",0,60);assert(have_command);
 ms++;control();yaw_record("watchdog_at",0,60);
 assert(!have_command&&!body.yaw.integral);
 for(unsigned i=0;i<4;i++)assert(!duty[i]);
}
static int yaw_suite(const char *output){
 yaw_trace=fopen(output,"w");assert(yaw_trace);
 fputs("case,step,ms,requested_w_mrad_s,measured_w_rad_s,yaw_I_m_s,linear_I_m_s,yaw_correction_m_s,fault,fault_source,source,target_A,target_B,target_C,target_D,inner_I_A,inner_I_B,inner_I_C,inner_I_D,duty_norm_A,duty_norm_B,duty_norm_C,duty_norm_D,duty_raw_A,duty_raw_B,duty_raw_C,duty_raw_D,CCR10_1,CCR11_1,CCR9_1,CCR9_2,CCR1_1,CCR1_2,CCR1_3,CCR1_4\n",yaw_trace);
 const int cases[8][2]={{60,-52},{-60,52},{60,-60},{-60,60},
                       {-60,54},{60,-54},{52,-52},{-52,52}};
 for(unsigned i=0;i<8;i++)yaw_reverse(cases[i][0],cases[i][1]);
 for(unsigned i=0;i<3;i++)for(int sign=-1;sign<=1;sign+=2){
  int w=(i==0?52:(i==1?60:100));yaw_hold(sign*w);
  yaw_periodic(w,50,sign);yaw_periodic(w,200,sign);
 }
 yaw_amplitude(-1);yaw_amplitude(1);
 yaw_braking(60,300);yaw_braking(-60,300);
 yaw_braking(-60,-300);yaw_braking(60,-300);
 yaw_straight_and_arcs();yaw_reset_and_limits();
 assert(!fclose(yaw_trace));yaw_trace=NULL;
 printf("YAW_REVERSAL_ACCEPTANCE failures=%u\n",yaw_failures);
 return yaw_failures?2:0;
}

/* Catch a stale opposite yaw I suppressing a real pure-turn reversal.
 * No motor/plant model: only actual firmware targets and timer commands. */
static unsigned reversal_checks, reversal_failures;
static void reversal_check(int ok,const char *name){
 reversal_checks++;
 if(!ok){reversal_failures++;printf("FAIL reversal %s\n",name);}
}
static void partial_reversal(int sign,unsigned history){
 yaw_ready();
 for(unsigned n=0;n<history;n++)yaw_step(0,sign*80);
 for(unsigned n=0;n<10;n++){
  yaw_step(0,-sign*80);
  /* Hand calculation at rest after rejecting old opposite I:
   * |wheel_ref|=(.08*.16 + .1*.08 + .2*.08*.01)*1000 -> 20 mm/s.
   * The new request must not be suppressed by its opposite history. */
  for(unsigned i=0;i<4;i++){
   int side=(i==0||i==2)?1:-1;
   reversal_check(targets[i]*(-sign)*side>=20,"partial_history_target");
   reversal_check(duty[i]*pwm_sign[i]*(-sign)*side>=80,"partial_history_output");
  }
 }
}
static void ramp_reversal(int sign){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 for(unsigned n=0;n<5;n++)yaw_step(0,sign*28);
 yaw_step(0,-sign*24);
 for(unsigned i=0;i<4;i++){
  int side=(i==0||i==2)?1:-1;
  /* 24 mrad/s fresh request -> 6 mm/s, then final 80 -> >=20. */
  reversal_check(targets[i]*(-sign)*side>=6,"ramped_first_new_direction");
 }
 for(unsigned n=0;n<5;n++)yaw_step(0,-sign*24);
 yaw_step(0,-sign*80);
 for(unsigned i=0;i<4;i++){
  int side=(i==0||i==2)?1:-1;
  reversal_check(targets[i]*(-sign)*side>=20,"ramped_full_request");
 }
}
static void reversal_braking(int sign){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 yaw_outer_step(0,-sign*.08f,0,sign*.30f);
 reversal_check(sign*(targets[0]-targets[1])<0,"brake_old_rotation");
 /* Also brake overspeed already in the newly requested direction.
  * The correction may legitimately oppose the target direction. */
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 yaw_outer_step(0,-sign*.08f,0,-sign*.30f);
 reversal_check(sign*(targets[0]-targets[1])>0,"brake_new_direction_overspeed");
}
static void retain_useful_yaw_integral(int sign){
 yaw_ready();
 for(unsigned n=0;n<100;n++)yaw_outer_step(0,sign*.08f,0,sign*.16f);
 float useful=body.yaw.integral;
 yaw_outer_step(0,-sign*.08f,0,0);
 reversal_check(body.yaw.integral*(-sign)>=useful*(-sign),
                "retain_integral_already_helping_new_direction");
}
static void no_repeated_reset(void){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,80);
 yaw_step(0,-80);
 for(unsigned n=0;n<100;n++)yaw_step(0,-80);
 reversal_check(targets[0]<=-36,"same_request_integral_can_build");
}

static void reversal_saturation_and_linear_history(int sign){
 yaw_ready();for(unsigned n=0;n<120;n++)yaw_step(0,sign*80);
 body_update(0,-sign*3.5f,0,0,.01f,targets);
 reversal_check(body.yaw.integral==0,"first_reversal_saturation_does_not_restore_old_I");
 for(unsigned i=0;i<4;i++)reversal_check(absi(targets[i])<=800,"reversal_wheel_limit");
 yaw_ready();
 for(unsigned n=0;n<100;n++)yaw_outer_step(.2f,0,.1f,0);
 for(unsigned n=0;n<120;n++)yaw_outer_step(0,sign*.08f,0,0);
 float retained=body.linear.integral;
 yaw_outer_step(0,-sign*.08f,0,0);
 reversal_check(body.linear.integral==retained&&retained>0,"linear_history_unchanged");
 reversal_check(sign*(targets[1]-targets[0])>=40,"yaw_differential_with_linear_history");
}

static int reversal_suite(void){
 const unsigned histories[]={20,60,100,120,180,200};
 for(int sign=-1;sign<=1;sign+=2){
  for(unsigned i=0;i<sizeof(histories)/sizeof(histories[0]);i++)
   partial_reversal(sign,histories[i]);
  ramp_reversal(sign);reversal_braking(sign);retain_useful_yaw_integral(sign);
  reversal_saturation_and_linear_history(sign);
 }
 no_repeated_reset();
 printf("YAW_PARTIAL_HISTORY checks=%u failures=%u\n",reversal_checks,reversal_failures);
 return reversal_failures?2:0;
}

int main(int argc,char **argv){
 assert(crc((const uint8_t*)"123456789",9)==0x29b1);
 imu_and_pid();protocol_and_arbitration();telemetry_and_legacy();bridge_and_saturation();
 puts("BODY_PID_IMU_CONVERSION_POLARITY_ARBITRATION_WATCHDOG_PROTOCOL_PASS");
 if(argc==2&&!strcmp(argv[1],"--reversal-regression"))return reversal_suite();
 if(argc==3&&!strcmp(argv[1],"--yaw-suite"))return yaw_suite(argv[2]);
 return reversal_suite();
}
