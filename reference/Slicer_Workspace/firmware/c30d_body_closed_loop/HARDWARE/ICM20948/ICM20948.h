#ifndef BODY_ICM20948_H
#define BODY_ICM20948_H
#include <stdint.h>
typedef struct {
 uint8_t who, present, initialized, calibrated, failed, have_sample;
 int16_t gyro_raw[3],accel_raw[3],gyro_mrad_s[3],accel_mm_s2[3];
 uint32_t sample_ms,sample_seq,bias_count;
 float yaw_rate,bias_sum,bias_raw;
} ICM20948State;
extern ICM20948State imu;
void icm20948_init(void);
void icm20948_sample(uint32_t now,const int16_t wheel_mm_s[4],int idle);
int icm20948_ready(uint32_t now);
#endif
