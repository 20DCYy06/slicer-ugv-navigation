/* UART扩展载荷：复用main.c现有UART/CRC外层，禁止直接发送C结构体内存，
 * 避免编译器对齐、float格式和大小端造成接口不一致。 */
#include "usartx.h"
#include "generated.h"
static uint16_t read16(const uint8_t*p){return p[0]|((uint16_t)p[1]<<8);}
static uint32_t read32(const uint8_t*p){return read16(p)|((uint32_t)read16(p+2)<<16);}
static void put16(uint8_t*p,uint16_t v){p[0]=v;p[1]=v>>8;}
static void put32(uint8_t*p,uint32_t v){put16(p,v);put16(p+2,v>>16);}
static unsigned absolute(int v){return v<0?(unsigned)-v:(unsigned)v;}
int body_decode_command(const uint8_t*p,unsigned size,uint32_t nonce,BodyCommand*out){
 if(size!=8||read32(p)!=nonce)return 0;
 int16_t v=(int16_t)read16(p+4),w=(int16_t)read16(p+6);
 if(absolute(v)>MAX_BODY_MM_S||absolute(w)>MAX_YAW_MRAD_S)return 0;
 out->vx_mm_s=v;out->wz_mrad_s=w;return 1;
}
void body_pack_telemetry(uint8_t out[64],const BodyTelemetry*in){
 out[0]=1;out[1]=in->imu_flags;out[2]=in->who;out[3]=in->source;put32(out+4,in->mcu_us);
 for(unsigned i=0;i<4;i++){put16(out+8+2*i,in->delta[i]);put16(out+16+2*i,in->wheel_mm_s[i]);}
 for(unsigned i=0;i<3;i++){
  put16(out+24+2*i,in->gyro_raw[i]);put16(out+30+2*i,in->accel_raw[i]);
  put16(out+36+2*i,in->gyro_mrad_s[i]);put16(out+42+2*i,in->accel_mm_s2[i]);
 }
 put16(out+48,in->measured_vx);put16(out+50,in->measured_wz);
 put16(out+52,in->target_vx);put16(out+54,in->target_wz);
 put16(out+56,in->battery_mv);put16(out+58,in->status_flags);
 put16(out+60,in->sample_age_ms);put16(out+62,in->ack_seq);
}
