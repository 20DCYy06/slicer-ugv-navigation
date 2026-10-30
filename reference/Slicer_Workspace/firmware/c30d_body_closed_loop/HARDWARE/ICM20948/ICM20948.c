/* 参照厂家 HARDWARE/I2C.* 与 ICM20948.c 的PB10/PB11和寄存器配置。
 * 适配当前无FreeRTOS的SLC2固件：有界I2C访问、12字节突发读取；不启用磁力计/DMP。
 * 量程固定为±500deg/s、±2g，物理量显式换算为ROS采用的rad/s、m/s²。 */
#include "ICM20948.h"
#include "generated.h"
#ifndef SLC2_NATIVE_TEST
#include "stm32f4xx.h"
#endif
ICM20948State imu;
#ifndef SLC2_NATIVE_TEST
static void delay_us(unsigned us){
 for(unsigned n=0;n<us;n++)for(volatile unsigned k=0;k<3;k++)__NOP();
}
static void sda(int high){if(high)GPIOB->BSRRL=1u<<11;else GPIOB->BSRRH=1u<<11;}
static void scl_low(void){GPIOB->BSRRH=1u<<10;}
static int scl_high(void){
 GPIOB->BSRRL=1u<<10;
 for(unsigned n=0;n<100;n++){if(GPIOB->IDR&(1u<<10))return 1;delay_us(1);}
 return 0;
}
static int start(void){sda(1);if(!scl_high())return 0;delay_us(2);sda(0);delay_us(2);scl_low();return 1;}
static void finish(void){sda(0);delay_us(2);(void)scl_high();delay_us(2);sda(1);}
static int write_byte(uint8_t value){
 for(unsigned n=0;n<8;n++){
  sda(value&0x80);value<<=1;delay_us(2);
  if(!scl_high())return 0;delay_us(2);scl_low();
 }
 sda(1);delay_us(2);if(!scl_high())return 0;delay_us(2);
 int ack=!(GPIOB->IDR&(1u<<11));scl_low();return ack;
}
static int read_byte(uint8_t*out,int ack){
 uint8_t value=0;sda(1);
 for(unsigned n=0;n<8;n++){
  delay_us(2);if(!scl_high())return 0;delay_us(2);
  value=(value<<1)|((GPIOB->IDR>>11)&1);scl_low();
 }
 sda(!ack);delay_us(2);if(!scl_high())return 0;delay_us(2);scl_low();sda(1);*out=value;return 1;
}
static int reg_write(uint8_t reg,uint8_t value){
 int ok=start()&&write_byte(IMU_ADDRESS<<1)&&write_byte(reg)&&write_byte(value);
 finish();return ok;
}
static int reg_read(uint8_t reg,uint8_t*out,unsigned size){
 int ok=start()&&write_byte(IMU_ADDRESS<<1)&&write_byte(reg)&&start()&&write_byte((IMU_ADDRESS<<1)|1);
 for(unsigned n=0;ok&&n<size;n++)ok=read_byte(out+n,n+1<size);
 finish();return ok;
}
#else
/* 测试只替换总线，实际初始化、量程换算和零偏滤波仍执行同一份代码。 */
extern int test_imu_write(uint8_t reg,uint8_t value);
extern int test_imu_read(uint8_t reg,uint8_t*out,unsigned size);
#define reg_write test_imu_write
#define reg_read test_imu_read
static void delay_us(unsigned us){(void)us;}
#endif
static float absolute(float x){return x<0?-x:x;}
static int16_t rounded(float x){return (int16_t)(x+(x<0?-.5f:.5f));}
void icm20948_init(void){
 imu=(ICM20948State){0};
#ifndef SLC2_NATIVE_TEST
 RCC->AHB1ENR|=1u<<1;(void)RCC->AHB1ENR;
 /* 开漏输出+上拉，释放高电平用于ACK、读取和时钟拉伸检测。 */
 for(unsigned p=10;p<=11;p++){
  GPIOB->MODER=(GPIOB->MODER&~(3u<<(2*p)))|(1u<<(2*p));
  GPIOB->OTYPER|=1u<<p;
  GPIOB->PUPDR=(GPIOB->PUPDR&~(3u<<(2*p)))|(1u<<(2*p));
 }
 sda(1);(void)scl_high();
 /* MCU复位不一定给IMU断电；释放可能停在半帧的I2C总线，再产生STOP。
  * 仅上电执行一次，不给运动控制增加过滤或等待规则。 */
 for(unsigned n=0;n<9;n++){
  scl_low();delay_us(2);
  if(!scl_high()){imu.failed=1;return;}
  delay_us(2);
 }
 finish();
#endif
 /* 冷上电时先等待传感器供电/内部启动稳定，不能立即读取WHO_AM_I。 */
 delay_us(100000);
#ifndef SLC2_NATIVE_TEST
 /* 此时尚未进入控制主循环、电机保持零输出；初始化分段喂狗，
  * 不修改运行期500ms看门狗，也不在运动控制中掩盖阻塞。 */
 IWDG->KR=0xaaaa;
#endif
 if(!reg_write(0x7f,0)||!reg_read(0x00,&imu.who,1)||imu.who!=0xea){imu.failed=1;return;}
 imu.present=1;
 if(!reg_write(0x06,0x80)){imu.failed=1;return;}delay_us(100000);
 int ok=reg_write(0x7f,0)&&reg_write(0x06,1)&&reg_write(0x07,0)&&
  reg_write(0x7f,0x20)&&reg_write(0x00,10)&&reg_write(0x01,0x33)&&
  reg_write(0x10,0)&&reg_write(0x11,10)&&reg_write(0x14,0x31)&&
  reg_write(0x7f,0)&&reg_write(0x11,1);
 imu.failed=!ok;imu.initialized=ok;
}
void icm20948_sample(uint32_t now,const int16_t wheels[4],int idle){
 if(!imu.initialized)return;
 uint8_t ready=0,data[12];
 if(!reg_read(0x1a,&ready,1)){imu.failed=1;return;}
 if(!(ready&1))return;
 if(!reg_read(0x2d,data,12)){imu.failed=1;return;}
 imu.failed=0;
 const int gs[3]=IMU_GYRO_SIGNS,as[3]=IMU_ACCEL_SIGNS;
 for(unsigned n=0;n<3;n++){
  imu.accel_raw[n]=(int16_t)(((unsigned)data[2*n]<<8)|data[2*n+1]);
  imu.gyro_raw[n]=(int16_t)(((unsigned)data[6+2*n]<<8)|data[7+2*n]);
 }
 /* 仅在无运动指令、轮子静止且陀螺变化很小时估计开机Z零偏。 */
 int stationary=idle;
 for(unsigned n=0;n<4;n++)if(absolute(wheels[n])>IMU_BIAS_WHEEL_MM_S)stationary=0;
 float raw=imu.gyro_raw[IMU_YAW_AXIS];
 if(absolute(raw*IMU_RAD_PER_COUNT)>IMU_BIAS_MAX_RAD_S)stationary=0;
 if(!imu.calibrated){
  if(stationary){imu.bias_sum+=raw;imu.bias_count++;}
  else{imu.bias_sum=0;imu.bias_count=0;}
  if(imu.bias_count>=IMU_BIAS_SAMPLES){imu.bias_raw=imu.bias_sum/imu.bias_count;imu.calibrated=1;}
 }
 for(unsigned n=0;n<3;n++){
  float corrected=imu.gyro_raw[n]-(n==IMU_YAW_AXIS?imu.bias_raw:0);
  imu.gyro_mrad_s[n]=rounded(corrected*IMU_RAD_PER_COUNT*1000.0f*gs[n]);
  imu.accel_mm_s2[n]=rounded(imu.accel_raw[n]*(9.80665f*1000.0f/16384.0f)*as[n]);
 }
 float dt=imu.have_sample?(now-imu.sample_ms)*.001f:CONTROL_MS*.001f;
 float rate=(raw-imu.bias_raw)*IMU_RAD_PER_COUNT*gs[IMU_YAW_AXIS];
 float alpha=dt/(IMU_FILTER_TAU_S+dt);
 imu.yaw_rate=imu.have_sample?imu.yaw_rate+alpha*(rate-imu.yaw_rate):rate;
 imu.sample_ms=now;imu.sample_seq++;imu.have_sample=1;
}
int icm20948_ready(uint32_t now){
 return imu.initialized&&imu.calibrated&&imu.have_sample&&(uint32_t)(now-imu.sample_ms)<=IMU_TIMEOUT_MS;
}
