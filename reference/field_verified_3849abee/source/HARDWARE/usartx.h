#ifndef BODY_USARTX_H
#define BODY_USARTX_H
#include <stdint.h>
/* SLC2原帧结构、CRC和旧命令不变；以下为新增类型，数值均小端。 */
#define BODY_COMMAND 0x16u    /* nonce:u32, vx_mm_s:i16, wz_mrad_s:i16 */
#define BODY_INFO_REQUEST 0x17u /* nonce:u32 */
#define BODY_INFO_REPLY 0x84u /* nonce:u32, version:u8, who:u8, imu_flags:u8, source:u8 */
#define BODY_TELEMETRY 0x93u  /* 64字节固定载荷，参见README表格 */
enum {SOURCE_STOP=0,SOURCE_WHEEL=1,SOURCE_PWM=2,SOURCE_BODY=3,SOURCE_CAL=4};
typedef struct {
 uint32_t nav_ms,manual_ms;
 uint8_t nav_mode,nav_valid,manual_valid;
 int16_t wheel[4],manual[4],vx_mm_s,wz_mrad_s;
} ControlSources;
typedef struct {int16_t vx_mm_s,wz_mrad_s;} BodyCommand;
typedef struct {
 uint8_t imu_flags,who,source;
 uint32_t mcu_us;
 int16_t delta[4],wheel_mm_s[4],gyro_raw[3],accel_raw[3],gyro_mrad_s[3],accel_mm_s2[3];
 int16_t measured_vx,measured_wz,target_vx,target_wz;
 uint16_t battery_mv,status_flags,sample_age_ms,ack_seq;
} BodyTelemetry;
int body_decode_command(const uint8_t*p,unsigned size,uint32_t nonce,BodyCommand*out);
void body_pack_telemetry(uint8_t out[64],const BodyTelemetry*in);
#endif
