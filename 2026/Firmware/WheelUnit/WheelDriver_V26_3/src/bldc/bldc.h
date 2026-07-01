#ifndef BLDC_H_
#define BLDC_H_

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "config.h"
#include "flash.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MAX_DUTY 0.99f                     // 最大デューティ比
#define MIN_DUTY 0.01f                     // 最小デューティ比
#define MAX_ADC_VAL 4095                   // ADCの最大値(12bit)
#define SPEED_LPF 0.5f                     // 角速度のローパスフィルタ係数
#define SPEED_LPF_INV 0.5f                 // (1.0 - SPEED_LPF) 事前計算値
#define SPEED_MEAS_MIN_DT 0.005f           // 速度計測の最小サンプリング間隔 [s]（量子化ノイズ低減）
#define K_SPEED_FF 0.052f                  // 速度フィードフォワードゲイン [V/(rad/s)] (=60/(Kv×2π), Kv=185rpm/V)
#define K_FRICTION_FF 0.2f                 // 静止摩擦補償フィードフォワード電圧 [V] (要実機調整)
#define FRICTION_FF_DEADBAND 0.5f          // 摩擦補償を有効にする目標角速度しきい値 [rad/s] (ゼロ点でのチャタリング防止)
#define AMP_LPF_COEF 0.5f                  // 振幅ローパスフィルタ係数
#define AMP_VOLT_LPF_COEF 0.5f             // 電圧振幅ローパスフィルタ係数
#define K_ADV 0.01f                        // 進角ゲイン
#define LOW_SPEED_GAIN_THRESHOLD 20.0f     // 速度ゲインスケジューリングのしきい値 [rad/s] (要実機調整)
#define LOW_SPEED_GAIN_MAX 5.0f            // 速度0付近でのkp/ki倍率 (要実機調整)
#define ADC2RADIAN 0.0015339807878856412f  // AC値をラジアンに変換する係数(2π/4096)

typedef struct {
  uint32_t max_encoder_val;    // エンコーダーの最大値
  uint32_t min_encoder_val;    // エンコーダーの最小値
  float encoder_offset_theta;  // エンコーダーのオフセット値
  uint32_t id;                 // モーターID (1-4)
} BLDCFlashData;

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float d_term;
  float d_lpf;
  float output_limit;
} PIDController;

typedef struct {
  uint32_t id;                 // モーターID (1-4)
  float dt;                    // 制御周期 [s]
  float amp;                   // 電圧振幅 [0 to 1]
  float amp_volt;              // 電圧振幅 [v]
  float encoder_offset_theta;  // エンコーダオフセット値
  uint16_t max_encoder_val;
  uint16_t min_encoder_val;
  float mech_theta;         // 機械角度 [rad]
  float elec_theta;         // 電気角度 [rad]
  float angular_speed;      // 角速度 [rad/s]
  float angular_accel;      // 角加速度 [rad/s^2]
  uint8_t pole_pairs;       // 極対数 (磁石の数/2)
  PIDController speed_pid;  // 角速度制御用PID
} SensoredVectorControl;

void BLDC_Init(bool do_set_encoder, uint32_t id, uint16_t* encoder_val);
void BLDC_Stop();
void BLDC_ForceLowSide(void);
void BLDC_OpenLoopDrive(float amp, float freq);
void BLDC_SensoredVectorControlDrive(uint16_t encoder_value, float supply_volt);
void BLDC_AngularSpeedControl(float target_angular_speed);
void BLDC_VoltageControl(float target_volt);

float BLDC_GetAngularSpeed(void);
float BLDC_GetAngularAccel(void);
float BLDC_GetMechTheta(void);
float BLDC_GetElecTheta(void);
float BLDC_GetAmpVolt(void);
uint32_t BLDC_GetId(void);

#endif  // BLDC_H_
