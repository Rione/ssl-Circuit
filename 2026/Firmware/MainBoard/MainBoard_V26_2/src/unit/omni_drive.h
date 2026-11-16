#ifndef __OMNI_DRIVE_H_
#define __OMNI_DRIVE_H_

#include <stdint.h>

#include "maf.h"
#include "mymath.h"
#include "parammeter.h"
#include "serial.h"
#include "timer.h"

typedef struct {
  int16_t vel_x;
  int16_t vel_y;
  int16_t motor_angular_velocity[4];
} OmniDriveStatus;

typedef struct {
  Serial* serials[4];
  float vel_wheel_angular[4];
  uint8_t emg;
  uint8_t ready;
  MAF maf[4];
  // OmniDrive_GetVelの逆運動学(最小二乗擬似逆行列)係数。ROBOT_MOTOR_DEGREEから
  // OmniDrive_Initで一度だけ計算してキャッシュする。
  float ik_vx[4];
  float ik_vy[4];
  float ik_omega[4];
} OmniDrive;

void OmniDrive_Init(OmniDrive* self, Serial* serials);
void OmniDrive_SetVel(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle);
void OmniDrive_SetFree(OmniDrive* self);
void OmniDrive_Send(OmniDrive* self, int16_t* m, uint8_t command);
void OmniDrive_Recv(OmniDrive* self);
void OmniDrive_GetVel(OmniDrive* self, int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle);

#endif  // __OMNI_DRIVE_H_
