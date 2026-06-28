#include "bldc.h"

static SensoredVectorControl svc;

static PwmOut u_pwm;
static PwmOut v_pwm;
static PwmOut w_pwm;

static Timer dt_timer;
static Timer speed_dt_timer;
static Timer accel_dt_timer;

static inline void BLDC_GetEncoder(uint16_t encoder_val) {
  uint16_t clamped_encoder_val = Constrain(encoder_val, svc.min_encoder_val, svc.max_encoder_val);
  float encoder_range = (float)(svc.max_encoder_val - svc.min_encoder_val);
  float theta = ((float)(clamped_encoder_val - svc.min_encoder_val) / encoder_range) * TWO_PI;
  svc.mech_theta = NormalizeRadians(theta - svc.encoder_offset_theta);
}

static inline void BLDC_CalculateAngularSpeed(void) {
  static float pre_speed = 0;
  static float pre_theta = 0;
  static float pre_delta_theta = 0;

  float dt = Timer_Read(&speed_dt_timer);
  if (dt < SPEED_MEAS_MIN_DT) return;  // 低速時の量子化ノイズ低減のため計測間隔を延長
  if (dt > 0.05f) dt = 0.05f;
  Timer_Reset(&speed_dt_timer);

  if (pre_theta == svc.mech_theta) {
    pre_theta = svc.mech_theta + pre_delta_theta;
    return;
  }

  float delta_theta = svc.mech_theta - pre_theta;

  // 0と2πの境目を跨いだ場合の補正
  if (delta_theta > PI) delta_theta -= TWO_PI;
  if (delta_theta < -PI) delta_theta += TWO_PI;

  // ノイズによる速度の異常値を補正（物理的に不可能な変位はノイズとして棄却）
  // 補正後のdelta_thetaは必ず[-π,π]に収まるため"> PI"では機能しなかった(dead code)
  float max_delta = MAX_ANGULAR_SPEED * dt;
  if (Abs(delta_theta) > max_delta) delta_theta = pre_delta_theta;
  pre_delta_theta = delta_theta;

  float speed = delta_theta / dt;
  speed = speed * SPEED_LPF_INV + pre_speed * SPEED_LPF;  // ローパスフィルタ

  svc.angular_speed = speed;
  pre_speed = speed;
  pre_theta = svc.mech_theta;
}

static inline float BLDC_PIDControl(PIDController* pid, float error, float dt) {
  // dtが不正(0や負)だと微分でNaNが出るため保護する
  if (dt <= 1e-6f) {
    // 比例項
    float p_term = pid->kp * error;

    // 積分項(微小なdtでも影響が小さいためそのまま累積)
    pid->integral += pid->ki * error * dt;
    pid->integral = Constrain(pid->integral, -pid->output_limit, pid->output_limit);

    // 微分項は更新しない
    pid->d_term = 0.0f;
    pid->prev_error = error;

    float output = p_term + pid->integral + pid->d_term;
    output = Constrain(output, -pid->output_limit, pid->output_limit);
    return output;
  }

  // 比例項
  float p_term = pid->kp * error;

  // 積分項
  pid->integral += pid->ki * error * dt;
  pid->integral = Constrain(pid->integral, -pid->output_limit, pid->output_limit);

  // 微分項
  float raw_d_term = pid->kd * (error - pid->prev_error) / dt;
  pid->d_term = pid->d_term * pid->d_lpf + raw_d_term * (1.0f - pid->d_lpf);
  pid->prev_error = error;

  // 出力の計算
  float output = p_term + pid->integral + pid->d_term;
  output = Constrain(output, -pid->output_limit, pid->output_limit);

  return output;
}

static inline void BLDC_SetEncoder(uint16_t* encoder_val) {
  svc.encoder_offset_theta = 0;  // オフセット計測前は0で初期化
  svc.max_encoder_val = 0;
  svc.min_encoder_val = MAX_ADC_VAL;

  // エンコーダー出力の最大値を取得する
  float phase = 0;
  for (uint16_t i = 0; i < 3000; i++) {
    phase += 0.2;
    if (svc.max_encoder_val < *encoder_val) svc.max_encoder_val = *encoder_val;
    if (svc.min_encoder_val > *encoder_val) svc.min_encoder_val = *encoder_val;
    BLDC_OpenLoopDrive(0.15, phase);
    HAL_Delay(1);
  }

  // 電気角度を0にオフセットする
  float offset_sum = 0;
  for (uint8_t i = 0; i < POLE_PAIRS; i++) {
    float theta_sum = 0;
    BLDC_OpenLoopDrive(0.2, 0);
    HAL_Delay(200);
    BLDC_OpenLoopDrive(0.4, 0);
    HAL_Delay(100);
    for (uint16_t i = 0; i < 200; i++) {
      BLDC_GetEncoder(*encoder_val);
      theta_sum += svc.mech_theta;
      HAL_Delay(1);
    }
    offset_sum += theta_sum * 0.005f;

    phase = 0;
    for (uint16_t i = 0; i < (TWO_PI * 10); i++) {
      phase += 0.1;
      BLDC_OpenLoopDrive(0.15, phase);
      HAL_Delay(1);
    }
  }
  svc.encoder_offset_theta = offset_sum / POLE_PAIRS;

  printf("(Measure)max_encoder_val: %lu, min_encoder_val: %lu, encoder_offset_theta: %.6f\n",
         (unsigned long)svc.max_encoder_val,
         (unsigned long)svc.min_encoder_val,
         svc.encoder_offset_theta);
}

static inline void BLDC_WritePwm(float u, float v, float w) {
  u = Constrain(u, MIN_DUTY, MAX_DUTY);
  v = Constrain(v, MIN_DUTY, MAX_DUTY);
  w = Constrain(w, MIN_DUTY, MAX_DUTY);

  PwmOut_Write(&u_pwm, u);
  PwmOut_Write(&v_pwm, v);
  PwmOut_Write(&w_pwm, w);
}

void BLDC_Init(bool do_set_encoder, uint32_t id, uint16_t* encoder_val) {
  printf("BLDC_Init\n");
  PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
  PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
  PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);

  Timer_Init(&dt_timer);
  Timer_Init(&speed_dt_timer);
  Timer_Init(&accel_dt_timer);  // フラッシュから読み込み

  // フラッシュに書き込み
  if (do_set_encoder) {
    printf("BLDC_EncoderOffset\n");
    BLDCFlashData read_data;
    Flash_ReadData(FLASH_USER_START_ADDR, &read_data, sizeof(read_data));
    printf("(From Flash)id: %u\n", read_data.id);
    svc.id = read_data.id;
    BLDC_SetEncoder(encoder_val);
    BLDCFlashData write_data = {svc.max_encoder_val, svc.min_encoder_val, (float)svc.encoder_offset_theta, (id == 0) ? svc.id : id};
    if (Flash_WriteData(FLASH_USER_START_ADDR, &write_data, sizeof(write_data)) != HAL_OK) {
      printf("Flash write error\n");
    }
  }

  // フラッシュから読み込み
  BLDCFlashData read_data;
  Flash_ReadData(FLASH_USER_START_ADDR, &read_data, sizeof(read_data));
  printf("(From Flash)max_encoder_val: %lu, min_encoder_val: %lu, encoder_offset_theta: %.6f, id: %u\n",
         (unsigned long)read_data.max_encoder_val,
         (unsigned long)read_data.min_encoder_val,
         read_data.encoder_offset_theta,
         read_data.id);

  svc.max_encoder_val = (uint16_t)read_data.max_encoder_val;
  svc.min_encoder_val = (uint16_t)read_data.min_encoder_val;
  svc.encoder_offset_theta = read_data.encoder_offset_theta;
  svc.id = read_data.id;

  // PIDコントローラ
  // 角速度制御
  svc.speed_pid.kp = 0.03;
  svc.speed_pid.ki = 0.5;
  svc.speed_pid.kd = 0;
  svc.speed_pid.d_term = 0;
  svc.speed_pid.d_lpf = 0.0f;
  svc.speed_pid.output_limit = MAX_AMP_VOLT;
}

void BLDC_Stop() {
  svc.amp = 0;
  svc.amp_volt = 0;
  // アンチワインドアップ:停止中はPID積分項を積み上げない。
  // これをしないと、ゲートドライバ無効中などモーターが動けない間に
  // 積分項が飽和し、駆動開始の瞬間にフル電圧が出て急加速する。
  svc.speed_pid.integral = 0;
  svc.speed_pid.prev_error = 0;
  svc.speed_pid.d_term = 0;
}

// 全相をローサイドONにする(ハイサイド完全OFF, duty=0)。
// ブートストラップ・コンデンサの充電に使う。MIN_DUTYのクランプを通さず生の0を書く。
void BLDC_ForceLowSide(void) {
  PwmOut_Write(&u_pwm, 0.0f);
  PwmOut_Write(&v_pwm, 0.0f);
  PwmOut_Write(&w_pwm, 0.0f);
}

void BLDC_OpenLoopDrive(float amp, float phase) {
  phase = NormalizeRadians(phase);

  float u = 0.5 + 0.5 * amp * Cos(phase);
  float v = 0.5 + 0.5 * amp * Cos(phase - TWO_THIRDS_PI);
  float w = 0.5 + 0.5 * amp * Cos(phase + TWO_THIRDS_PI);

  BLDC_WritePwm(u, v, w);
}

void BLDC_SensoredVectorControlDrive(uint16_t encoder_value, float supply_volt) {
  // 制御周期の取得
  svc.dt = Timer_Read(&dt_timer);
  Timer_Reset(&dt_timer);
  if (svc.dt > 0.001) svc.dt = 0.001;  // 制御周期が大きすぎる場合は無視

  // エンコーダ値を処理
  BLDC_GetEncoder(encoder_value);  // ラジアン(0〜2π)に変換
  BLDC_CalculateAngularSpeed();    // 角速度を計算

  // 電気角度を計算
  svc.elec_theta = svc.mech_theta * POLE_PAIRS;                       // 電気角度 = 機械角度 * 極対数 + 位相合わせオフセット
  svc.elec_theta += Constrain(svc.angular_speed * K_ADV, -1.5, 1.5);  // 進角を加算(これがあると高速回転時に安定する)
  svc.elec_theta = NormalizeRadians(svc.elec_theta);

  svc.amp = svc.amp * AMP_LPF_COEF + (svc.amp_volt / supply_volt) * AMP_VOLT_LPF_COEF;
  svc.amp = Constrain(svc.amp, -0.5, 0.5);

  // SVPWM (min-max offset injection)
  float u_ref = svc.amp * Sin(svc.elec_theta);
  float v_ref = svc.amp * Sin(svc.elec_theta - TWO_THIRDS_PI);
  float w_ref = svc.amp * Sin(svc.elec_theta + TWO_THIRDS_PI);
  float v_max = u_ref > v_ref ? (u_ref > w_ref ? u_ref : w_ref) : (v_ref > w_ref ? v_ref : w_ref);
  float v_min = u_ref < v_ref ? (u_ref < w_ref ? u_ref : w_ref) : (v_ref < w_ref ? v_ref : w_ref);
  float offset = 0.5f * (v_max + v_min);
  float u = 0.5f + u_ref - offset;
  float v = 0.5f + v_ref - offset;
  float w = 0.5f + w_ref - offset;
  BLDC_WritePwm(u, v, w);
}

void BLDC_AngularSpeedControl(float target_angular_speed) {
  // 最大角速度制限
  target_angular_speed = Constrain(target_angular_speed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

  // 最大角加速度制限
  static float prev_target_angular_speed = 0;
  // svc.dtが0の場合は制御をスキップしてNaN発生を防ぐ
  if (svc.dt <= 1e-6f) {
    return;
  }

  float accel = (target_angular_speed - prev_target_angular_speed) / svc.dt;
  accel = Constrain(accel, -MAX_ANGULAR_ACCEL, MAX_ANGULAR_ACCEL);
  target_angular_speed = prev_target_angular_speed + accel * svc.dt;
  prev_target_angular_speed = target_angular_speed;

  // 低速ゲインスケジューリング: 低速域は逆起電力によるフィードフォワードが効きにくく、
  // 静止摩擦の補償をPIDのフィードバックだけで背負うため、目標値が小さいほどkp/kiを連続的に増幅する。
  // ノイズが乗った測定速度ではなく目標値(スルーレート制限後)でスケジューリングすることで、
  // 速度ノイズがゲイン自体を揺らして発振を助長する事態を避けている。
  float speed_ratio = Constrain(Abs(target_angular_speed) / LOW_SPEED_GAIN_THRESHOLD, 0.0f, 1.0f);
  float gain_scale = LOW_SPEED_GAIN_MAX - (LOW_SPEED_GAIN_MAX - 1.0f) * speed_ratio;

  PIDController scaled_pid = svc.speed_pid;
  // scaled_pid.kp *= gain_scale;
  scaled_pid.ki *= gain_scale;

  float pid_out = BLDC_PIDControl(&scaled_pid, target_angular_speed - svc.angular_speed, svc.dt);
  svc.speed_pid.integral = scaled_pid.integral;
  svc.speed_pid.prev_error = scaled_pid.prev_error;
  svc.speed_pid.d_term = scaled_pid.d_term;

  svc.amp_volt = Constrain(pid_out + target_angular_speed * K_SPEED_FF, -MAX_AMP_VOLT, MAX_AMP_VOLT);
}

void BLDC_VoltageControl(float target_volt) {
  svc.amp_volt = target_volt;
}

float BLDC_GetAngularSpeed(void) { return svc.angular_speed; }
float BLDC_GetAngularAccel(void) { return svc.angular_accel; }
float BLDC_GetMechTheta(void) { return svc.mech_theta; }
float BLDC_GetElecTheta(void) { return svc.elec_theta; }
float BLDC_GetAmpVolt(void) { return svc.amp_volt; }
uint32_t BLDC_GetId(void) { return svc.id; }
