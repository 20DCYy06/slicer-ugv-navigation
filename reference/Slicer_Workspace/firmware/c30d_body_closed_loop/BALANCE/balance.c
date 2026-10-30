/* 车身外环：编码器均速约束前进速度，ICM20948约束实际偏航角速度。
 * 内环仍由 main.c 的现有四轮编码器PI执行，不在这里叠加PWM死区。 */
#include "balance.h"
#include "generated.h"
BodyControl body;
static float limit(float x,float n){return x>n?n:(x < -n?-n:x);}
static float absolute(float x){return x<0?-x:x;}
void body_reset(void){
 body.linear=(BodyPID){0};body.yaw=(BodyPID){0};
 body.linear_correction=body.yaw_correction=0;
}
static float pid(BodyPID*s,float error,float dt,float kp,float ki,float kd,float cap){
 float derivative=s->started?(error-s->previous_error)/dt:0;
 float proposed=limit(s->integral+ki*error*dt,cap);
 float total=kp*error+proposed+kd*derivative;
 /* 输出已饱和且误差使其继续增大时，冻结积分，允许反向误差退饱和。 */
 if(!((total>cap&&error>0)||(total < -cap&&error<0)))s->integral=proposed;
 s->previous_error=error;s->started=1;
 return limit(kp*error+s->integral+kd*derivative,cap);
}
void body_update(float vx,float wz,float measured_vx,float measured_wz,float dt,
                 int16_t wheels[4]){
 /* A pure-turn request reversal invalidates only an opposing yaw I.
  * Do this once at the command boundary, before the saturation snapshot.
  * Keep feedback P/D (including braking), linear I and the wheel PI intact. */
 if(vx==0&&body.target_vx==0&&wz*body.target_yaw<0&&body.yaw.integral*wz<0)
  body.yaw.integral=0;
 body.target_vx=vx;body.target_yaw=wz;
 body.measured_vx=measured_vx;body.measured_yaw=measured_wz;
 if(vx==0&&wz==0){body_reset();for(unsigned i=0;i<4;i++)wheels[i]=0;return;}
 float old_v=body.linear.integral,old_w=body.yaw.integral;
 float yaw_error=wz-measured_wz;
 float yaw_pd=BODY_W_KP*yaw_error+BODY_W_KD*
  (body.yaw.started?(yaw_error-body.yaw.previous_error)/dt:0);
 body.linear_correction=pid(&body.linear,vx-measured_vx,dt,
  BODY_V_KP,BODY_V_KI,BODY_V_KD,BODY_V_CAP);
 body.yaw_correction=pid(&body.yaw,yaw_error,dt,
  BODY_W_KP,BODY_W_KI,BODY_W_KD,BODY_W_CAP);
 float forward=vx+body.linear_correction;
 /* 厂家四驱几何前馈，轮距/轴距来自配置；正wz使右轮快、左轮慢。 */
 float differential=wz*(TRACK_WIDTH_M+WHEELBASE_M)*0.5f+body.yaw_correction;
 /* Pure turn, feedback no faster than the request (both rad/s). If old I
  * alone reverses/cancels the differential, bound only that I at -PD (m/s),
  * retaining geometric feedforward. This is not a physical stop/start band.
  * Faster rotation, w=0 straight correction and moving arcs keep normal PID. */
 if(vx==0&&wz!=0&&absolute(measured_wz)<=absolute(wz)&&yaw_pd*wz>=0&&
    body.yaw.integral*wz<0&&differential*wz<=0){
  body.yaw.integral=limit(-yaw_pd,BODY_W_CAP);
  body.yaw_correction=limit(yaw_pd+body.yaw.integral,BODY_W_CAP);
  differential=wz*(TRACK_WIDTH_M+WHEELBASE_M)*0.5f+body.yaw_correction;
  old_w=body.yaw.integral; /* Saturation rollback must not restore rejected I. */
 }
 const int side[4]=WHEEL_SIDE_VALUES;
 float values[4],peak=MAX_MM_S;
 for(unsigned i=0;i<4;i++){
  values[i]=(forward+side[i]*differential)*1000.0f;
  if(absolute(values[i])>peak)peak=absolute(values[i]);
 }
 if(peak>MAX_MM_S){body.linear.integral=old_v;body.yaw.integral=old_w;}
 float scale=MAX_MM_S/peak;
 for(unsigned i=0;i<4;i++)wheels[i]=(int16_t)(values[i]*scale);
}
