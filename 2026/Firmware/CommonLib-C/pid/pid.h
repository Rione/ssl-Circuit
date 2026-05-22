#ifndef PID_H_
#define PID_H_

#include <stdint.h>

// PID制御ライブラリ

// PID動作モード
// PID_MODE_STD        : 標準PID
// PID_MODE_IPD        : 比例先行型PID（I-PD）
// PID_MODE_PDFEEDFWD  : 比例微分先行型PID
typedef enum {
      PID_MODE_STD = 0,
      PID_MODE_IPD,
      PID_MODE_PDFEEDFWD,
} PID_Mode;

// PID制御構造体

typedef struct {
      // ゲイン
      double Kp;  // 比例ゲイン
      double Ki;  // 積分ゲイン
      double Kd;  // 微分ゲイン

      // サンプリング周期 [s]
      double dt;

      // 動作モード
      PID_Mode mode;

      // 内部状態
      double prev_error;    // 前回偏差
      double prev_measure;  // 前回の測定値
      double integral;      // 積分値

      // 出力制限
      double out_min;  // 出力下限
      double out_max;  // 出力上限

      // 積分制限（アンチワインドアップ用）
      double i_min;  // 積分下限
      double i_max;  // 積分上限

} PID;

// PIDの初期化
static inline void PID_Init(PID* pid,
                            double Kp,
                            double Ki,
                            double Kd,
                            double dt,
                            PID_Mode mode) {
      pid->Kp = Kp;
      pid->Ki = Ki;
      pid->Kd = Kd;
      pid->dt = dt;
      pid->mode = mode;

      pid->prev_error = 0.0;
      pid->prev_measure = 0.0;
      pid->integral = 0.0;
}

// 出力の上下限を設定
static inline void PID_SetOutputLimit(PID* pid, double out_min, double out_max) {
      pid->out_min = out_min;
      pid->out_max = out_max;
}

// 積分項の上下限を設定（アンチワインドアップ制御）
static inline void PID_SetIntegralLimit(PID* pid, double i_min, double i_max) {
      pid->i_min = i_min;
      pid->i_max = i_max;
}

// PID内部状態をリセット
static inline void PID_Reset(PID* pid) {
      pid->prev_error = 0.0;
      pid->prev_measure = 0.0;
      pid->integral = 0.0;
}

// 制御出力を計算
static inline double PID_Update(PID* pid, double setpoint, double measure) {
      // 偏差
      double error = setpoint - measure;

      // 積分項の更新（全モード共通で偏差を積分）
      pid->integral += error * pid->dt;
      // 積分の飽和（アンチワインドアップ）
      if (pid->integral > pid->i_max) pid->integral = pid->i_max;
      if (pid->integral < pid->i_min) pid->integral = pid->i_min;

      // 微分項
      double derivative = 0.0;

      double u = 0.0;

      switch (pid->mode) {
            case PID_MODE_STD:
                  // 標準PID
                  derivative = (error - pid->prev_error) / pid->dt;
                  u = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
                  break;

            case PID_MODE_IPD:
                  // 比例先行型PID（I-PD）
                  derivative = (error - pid->prev_error) / pid->dt;
                  u = pid->Kp * setpoint - pid->Kp * measure + pid->Ki * pid->integral + pid->Kd * derivative;
                  break;

            case PID_MODE_PDFEEDFWD:
                  // 比例微分先行型PID（P+D先行）
                  derivative = (measure - pid->prev_measure) / pid->dt;  // dy/dt
                  u = pid->Kp * error - pid->Kd * derivative + pid->Ki * pid->integral;
                  break;

            default:
                  // 未定義モードの場合は標準PIDと同じ
                  derivative = (error - pid->prev_error) / pid->dt;
                  u = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
                  break;
      }

      // 次回用に状態を保存
      pid->prev_error = error;
      pid->prev_measure = measure;

      // 出力の飽和
      if (u > pid->out_max) u = pid->out_max;
      if (u < pid->out_min) u = pid->out_min;

      return u;
}

#endif  // PID_H_
