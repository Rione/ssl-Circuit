#ifndef BLDC_H_
#define BLDC_H_

#include <stdio.h>

#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MIN_DUTY 0.3
#define MAX_DUTY 0.7
#define MAX_ADC_VAL 4095                  // ADCの最大値(12bit)
#define ADC2RADIAN 0.0015339807878856412  // ADC値をラジアンに変換する係数(2π/4096)
#define MAX_SPEED 150.0f                  // 最大速度 [rad/s]
#define MAX_ACCEL 10.0f                   // 最大加速度 [rad/s^2]

typedef struct {
  uint32_t max_encoder_val;    // エンコーダーの最大値
  float encoder_offset_theta;  // エンコーダーのオフセット値
} BLDCFlashData;

// 構造体
typedef struct {
  double kp;
  double ki;
  double kd;
  double integral;
  double prev_error;
  double output_limit;
} PIDController;

typedef struct {
  double dt;                    // 制御周期 [s]
  double amp;                   // 電圧振幅 [0 to 1]
  double amp_volt;              // 電圧振幅 [v]
  double encoder_offset_theta;  // エンコーダオフセット値
  uint16_t max_encoder_val;
  double adc_correction_factor;
  double mech_theta;        // 機械角度 [rad]
  double elec_theta;        // 電気角度 [rad]
  double speed;             // 速度 [rad/s]
  uint8_t pole_pairs;       // 極対数 (磁石の数/2)
  PIDController speed_pid;  // 速度制御用PID
} SensoredVectorControl;

void BLDC_Init(SensoredVectorControl* svc, bool do_set_encoder, uint16_t* encoder_val);

void BLDC_Stop(bool brake);
void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, double supply_volt);

void BLDC_SpeedControl(SensoredVectorControl* svc, double target_speed);
void BLDC_OpenLoopDrive(double amp, double phase);

#endif  // BLDC_H_