#ifndef BODY_BALANCE_H
#define BODY_BALANCE_H
#include <stdint.h>
typedef struct {float integral, previous_error; uint8_t started;} BodyPID;
typedef struct {
 BodyPID linear, yaw;
 float measured_vx, measured_yaw, linear_correction, yaw_correction;
 float target_vx, target_yaw;
} BodyControl;
extern BodyControl body;
void body_reset(void);
void body_update(float vx,float wz,float measured_vx,float measured_wz,float dt,
                 int16_t wheels[4]);
#endif
