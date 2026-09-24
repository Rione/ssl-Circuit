#include "omni_drive.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "volt_tune.h"
#include "wheel_voltage.h"
// BugFixブランチの OmniDrive_ComputeInverseKinematics (ik_vx/vy/omega) は、狙いが同じ
// (車輪速度→機体速度の最小二乗疑似逆行列) だったため、下の fk/force_alloc に一本化した

static void OmniDrive_MonitorRx(OmniDrive* self);

// 逆運動学 H (4x3, 行 [-sinθi, cosθi, R]) から最小二乗疑似逆行列 (HᵀH)⁻¹Hᵀ を求める。
// 55°/135° 配置は Σsin²θ≠Σcos²θ, Σcosθ≠0 のため、単純な Σ/2 では vx が約17%過大になる
static void OmniDrive_ComputeForwardKinematics(OmniDrive* self) {
  float h[4][3];
  for (int i = 0; i < 4; i++) {
    h[i][0] = -SinDeg(ROBOT_MOTOR_DEGREE[i]);
    h[i][1] = CosDeg(ROBOT_MOTOR_DEGREE[i]);
    h[i][2] = ROBOT_WHEEL_BASE_RADIUS;
  }

  float a[3][3] = {{0}};
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      for (int i = 0; i < 4; i++) a[r][c] += h[i][r] * h[i][c];
    }
  }

  // 3x3 逆行列 (余因子展開)
  float inv[3][3];
  inv[0][0] = a[1][1] * a[2][2] - a[1][2] * a[2][1];
  inv[0][1] = a[0][2] * a[2][1] - a[0][1] * a[2][2];
  inv[0][2] = a[0][1] * a[1][2] - a[0][2] * a[1][1];
  inv[1][0] = a[1][2] * a[2][0] - a[1][0] * a[2][2];
  inv[1][1] = a[0][0] * a[2][2] - a[0][2] * a[2][0];
  inv[1][2] = a[0][2] * a[1][0] - a[0][0] * a[1][2];
  inv[2][0] = a[1][0] * a[2][1] - a[1][1] * a[2][0];
  inv[2][1] = a[0][1] * a[2][0] - a[0][0] * a[2][1];
  inv[2][2] = a[0][0] * a[1][1] - a[0][1] * a[1][0];
  float det = a[0][0] * inv[0][0] + a[0][1] * inv[1][0] + a[0][2] * inv[2][0];

  for (int r = 0; r < 3; r++) {
    for (int i = 0; i < 4; i++) {
      float sum = 0.0f;
      for (int c = 0; c < 3; c++) sum += inv[r][c] * h[i][c];
      self->fk[r][i] = sum / det;
    }
  }

  // 電圧制御の加減速ぶんとPIの補正を4輪に配る行列 (力の配分)。fkᵀ = H(HᵀH)⁻¹ は、機体の力と
  // モーメントを最小の力で4輪に配る。並進の列は Σsin²θ 倍して前後の列を従来の −sinθ と同じにする
  // (左右の列は約0.914 になり、従来の cosθ=0.574/0.707 の1.3〜1.6倍。55°/135°配置では左右の方が
  // 1輪あたり大きな力が要るため)。回転の列は 4R 倍して、従来の「4輪に同じだけ」と同じ大きさにする
  float sum_sin2 = 0.0f;
  for (int i = 0; i < 4; i++) sum_sin2 += h[i][0] * h[i][0];
  for (int i = 0; i < 4; i++) {
    self->force_alloc[i][0] = self->fk[0][i] * sum_sin2;
    self->force_alloc[i][1] = self->fk[1][i] * sum_sin2;
    self->force_alloc[i][2] = self->fk[2][i] * 4.0f * ROBOT_WHEEL_BASE_RADIUS;
  }
}

void OmniDrive_Init(OmniDrive* self, Serial* serials) {
  for (int i = 0; i < 4; i++) {
    self->serials[i] = &serials[i];
  }
  self->emg = 0;
  self->ready = 0;
  for (int i = 0; i < 4; i++) {
    self->vel_wheel_angular[i] = 0.0f;
    self->target_wheel_angular[i] = 0.0f;
    self->wheel_status[i] = 0;
    self->wheel_frame_count[i] = 0;
    self->wheel_rx_restart_count[i] = 0;
    self->wheel_last_frame_tick[i] = 0;
    self->wheel_rx_stalled[i] = false;
    self->cmd_voltage[i] = 0.0f;
    MAF_Init(&self->maf[i], 25);
  }
  self->rx_monitor_started = false;
  self->use_voltage_control = false;
  for (int k = 0; k < 3; k++) self->vel_fb_integral[k] = 0.0f;
  self->volt_saturated = false;
  self->volt_pi_saturated = false;
  self->volt_traction_limited = false;
#if OMNI_TX_AVOID_HEADER_BYTE
  // 応急処置が入ったFWであることをログで確認できるようにする (引き継ぎ文書 5.4)
  printf("# WORKAROUND: OMNI_TX_AVOID_HEADER_BYTE=1 (avoid 0xAA in WheelUnit TX data)\n");
#endif
  // フラッシュに保存された調整値 (自動最適化の結果) があれば、既定値にする
  if (VoltTune_LoadSaved()) {
    VoltTune_SetDefaults(&volt_tune);
    VoltTune_SetDefaults(&volt_tune_base);
    printf("# volt_tune: loaded saved tuning ka_lin=%d ka_lat=%d (x1000)\n", (int)(volt_tune.ka_lin * 1000.0f),
           (int)(volt_tune.ka_lat * 1000.0f));
  }
  if (VoltTune_Sanitize(&volt_tune)) printf("# volt_tune: out of range, corrected\n");
  OmniDrive_ComputeForwardKinematics(self);
  printf("# force_alloc [x,y,w] x1000:");
  for (int i = 0; i < 4; i++) {
    printf(" [%d,%d,%d]", (int)(self->force_alloc[i][0] * 1000.0f),
           (int)(self->force_alloc[i][1] * 1000.0f), (int)(self->force_alloc[i][2] * 1000.0f));
  }
  printf("\n");
  TCS_Init(&self->tcs);
}

void OmniDrive_SetVel(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle) {
  OmniDrive_SetVelEx(self, vel_x, vel_y, vel_angle, NULL);
}

// 呼び出し間隔の実測dt [s]。制御ループは基本1ms周期だが、他処理のブロッキング
// (例: Rock5A SPIストール検知時のprintf)で稀に周期が乱れることがある。
// 固定値dtを使うとTCSの加速度差分計算(オドメトリ速度の微分)がその分だけ実際の
// 数百倍の値になり、スリップ誤検知や速度平滑化の破綻を招くため、実測して使う。
static float OmniDrive_MeasureDt(void) {
  static Timer timer = {0};
  static bool initialized = false;
  const float kNominalDt = (float)ROBOT_CONTROL_LOOP_DT_US * 1e-6f;
  const float kMaxDt = 0.05f;  // 50ms超のギャップはクランプ (異常時の暴走防止)

  if (!initialized) {
    Timer_Init(&timer);
    Timer_Reset(&timer);
    initialized = true;
    return kNominalDt;
  }

  float dt = Timer_Read(&timer);
  Timer_Reset(&timer);
  if (dt <= 0.0f) dt = kNominalDt;
  if (dt > kMaxDt) dt = kMaxDt;
  return dt;
}

void OmniDrive_SetVelEx(OmniDrive* self, int16_t vel_x, int16_t vel_y, int16_t vel_angle,
                        const Imu* imu) {
  float vx_m = vel_x / 1000.0f;
  float vy_m = vel_y / 1000.0f;
  float omega_rad = vel_angle * 0.001f;
  const float dt = OmniDrive_MeasureDt();

  // 1. TCS: 対地速度推定 → スリップ検知 → スリップ量に応じたS字加減速
  TCSInput in;
  for (int i = 0; i < 4; i++) {
    in.wheel_vel[i] = self->vel_wheel_angular[i];
  }
  OmniDrive_GetVelF(self, &in.odom_vx, &in.odom_vy, &in.odom_omega);
  in.has_imu = (imu != NULL);
  in.gyro_yaw_rate = in.has_imu ? imu->yaw_rate : 0.0f;
  in.accel_x = in.has_imu ? imu->accel_robot_x : 0.0f;
  in.accel_y = in.has_imu ? imu->accel_robot_y : 0.0f;

  float cmd_vx, cmd_vy, cmd_omega;
  TCS_Update(&self->tcs, &in, vx_m, vy_m, omega_rad, &cmd_vx, &cmd_vy, &cmd_omega, dt);

  // 2. オムニホイール逆運動学: v_w = -vx*sin(θ) + vy*cos(θ) + R*ω
  // ※ バッテリー電圧補正は WheelUnit 側 (FOC に電源電圧を渡している) で行われる。
  //    ここで目標角速度を増やすと速度閉ループのため実速度が指令より速くなってしまう
  float target_wheel_angular[4];
  for (int i = 0; i < 4; i++) {
    float v_wheel_linear =
        -cmd_vx * SinDeg(ROBOT_MOTOR_DEGREE[i]) +
        cmd_vy * CosDeg(ROBOT_MOTOR_DEGREE[i]) +
        ROBOT_WHEEL_BASE_RADIUS * cmd_omega;

    target_wheel_angular[i] = v_wheel_linear / ROBOT_WHEEL_RADIUS;
    self->target_wheel_angular[i] = target_wheel_angular[i];  // ログ用 (クランプ前)
  }

  // 3a. 電圧制御: フィードフォワード (各輪の目標角速度・目標角加速度から) ＋ 機体速度のPI。
  //     PIは機体の3自由度 (vx, vy, ω) で誤差を取り、逆運動学と同じ向きで4輪に配るので、
  //     4輪が互いに逆らう成分 (押し合い) は出ない。並進の実速度はオドメトリなので、車輪が空転すると
  //     実速度が上がって電圧が下がる (そのままトラクション制御として働く)
  if (self->use_voltage_control) {
    const TractionControl* tcs = &self->tcs;
    const VoltTuneParams* vt = &volt_tune;  // 調整できる値 (volt_tune.h。既定値は parammeter.h)
    float meas_omega = (imu != NULL) ? imu->yaw_rate : in.odom_omega;
    float err[3] = {cmd_vx - in.odom_vx, cmd_vy - in.odom_vy, cmd_omega - meas_omega};
    // 積分: スリップ中 (オドメトリが当てにならない) は全軸止める。出力を縮めている間は並進を止める
    // (目標が実機より先に行って誤差が溜まり、加速の終わりで行き過ぎるのを防ぐ)。回転は、PIの分まで
    // 縮めたときだけ止める (向きを直す力を残しつつ、効かない間に溜まって後で振れるのを防ぐ)
    if (!tcs->is_slipping) {
      const float ki[3] = {vt->ki_lin, vt->ki_lin, vt->ki_ang};
      for (int k = 0; k < 3; k++) {
        if (k < 2 && self->volt_saturated) continue;
        if (k == 2 && self->volt_pi_saturated) continue;
        self->vel_fb_integral[k] = Constrain(self->vel_fb_integral[k] + ki[k] * err[k] * dt,
                                             -vt->i_max_v, vt->i_max_v);
      }
    }
    // 機体座標の補正電圧 [V]: force_alloc で4輪に配る (前後は −sinθ、左右は約0.914、回転は約1)
    float ux = vt->kp_lin * err[0] + self->vel_fb_integral[0];
    float uy = vt->kp_lin * err[1] + self->vel_fb_integral[1];
    float uw = vt->kp_ang * err[2] + self->vel_fb_integral[2];

    // 出力の整形: 各輪の電圧を「今の速度で転がり続けるための電圧 (center)」と、それを超える分
    // (加減速のトルクに当たる) に分ける。超える分はさらに FF の分 (目標の加減速) と PI の分 (向き・横ずれ・
    // 速度の誤差を直す) に分け、上限を超えそうなら **先に FF の分を** 4輪同じ比率で縮める。
    // それでも収まらないときだけ PI の分も同じ比率で縮める。
    //  - 上限は2つ: 電圧上限 (±WHEEL_VOLT_MAX) と、トルク上限 (±traction_limit_v。滑らずに出せる
    //    加速ぶんの電圧。TCS の accel_gain は掛けない)
    //  - 輪ごとに切り詰めると、その輪だけトルクが減って機体を回す力が生まれる (3.0m/s で逆転側の輪が
    //    電圧上限に張り付き、機体が半回転した)。同じ比率で縮めれば機体に掛かる力の向きは変わらない
    //  - PI も一緒に縮めていたときは、加速中ほぼずっと向き・横ずれの補正が弱まり、前後左右にぶれた
    //  - center は各輪の実際の回転数から出す (超える分＝モータトルクの上限になる)。
    //    IMUが無いとき (浮かせた確認) は center=0 とし、電圧上限だけで縮める
    bool use_traction_limit = (imu != NULL);

    float center[4], ff_excess[4], pi_excess[4], allowed[4];
    for (int i = 0; i < 4; i++) {
      // 加減速ぶんとPIの補正は、機体の加速度・補正を「力の配分」(force_alloc) で4輪に配る。
      // 以前は車輪の角加速度 (運動学) で配っていたため、左右は前後と同じ係数になり、1輪あたり約1.4倍の
      // 力が要る左右の加速が遅かった (前進1.5mで1.0m/sに241ms、左は421ms)。旋回も並進とは別の係数
      // (並進の係数を使うと回り始めに1輪約2Vかかり、指令の約1.8倍の速さで回った)
      const float* a = self->force_alloc[i];
      // x (前後) は ka_lin、y (左右) は ka_lat。斜めは両方の和
      float accel_volt = vt->ka_lin * a[0] * tcs->current_ax + vt->ka_lat * a[1] * tcs->current_ay +
                         vt->ka_ang * a[2] * tcs->current_alpha;
      // center は各輪の実際の回転数で転がり続ける電圧。超える分がそのままモータのトルク (電流) になる。
      // 以前は推定した対地速度から出していたが、減速中のスリップ判定の間は推定 (IMU積分のみ) が
      // 実速度より0.6m/sほど高いまま残り、ブレーキ側のトルクがほとんど出せず行き過ぎた
      center[i] = 0.0f;
      if (use_traction_limit) {
        center[i] = WheelVoltage_Feedforward(i, self->vel_wheel_angular[i]);
      }
      ff_excess[i] = WheelVoltage_Feedforward(i, target_wheel_angular[i]) + accel_volt - center[i];
      pi_excess[i] = a[0] * ux + a[1] * uy + a[2] * uw;
      // 超える分に許す大きさ (電圧上限までの余裕と、トルク上限の小さい方)
      allowed[i] = fmaxf(WHEEL_VOLT_MAX - fabsf(center[i]), 0.0f);
      if (use_traction_limit) allowed[i] = fminf(allowed[i], vt->traction_limit_v);
    }

    // 1) PI の分だけで上限を超える輪があれば、PI の分を縮める (このとき FF の分は0)
    float pi_scale = 1.0f;
    for (int i = 0; i < 4; i++) {
      float mag = fabsf(pi_excess[i]);
      if (mag > allowed[i]) pi_scale = fminf(pi_scale, allowed[i] / mag);
    }
    // 2) PI の分を残したうえで入れられる FF の分の最大の比率。|s·f + p| ≤ a を満たす s の上限
    //    (|p| ≤ a なので s=0 は必ず満たす)
    float ff_scale = 1.0f;
    for (int i = 0; i < 4; i++) {
      float f = ff_excess[i];
      if (fabsf(f) < 1e-6f) continue;
      float p = pi_scale * pi_excess[i];
      float hi = (f > 0.0f) ? (allowed[i] - p) / f : (-allowed[i] - p) / f;
      ff_scale = fminf(ff_scale, hi);
    }
    ff_scale = fmaxf(ff_scale, 0.0f);

    int16_t mv[4];
    for (int i = 0; i < 4; i++) {
      float volt = center[i] + ff_scale * ff_excess[i] + pi_scale * pi_excess[i];
      self->cmd_voltage[i] = Constrain(volt, -WHEEL_VOLT_MAX, WHEEL_VOLT_MAX);
      mv[i] = (int16_t)(self->cmd_voltage[i] * 100.0f);
    }
    // 縮めている間は積分を止める (次の周期で参照。並進は FF か PI を縮めたとき、回転は PI を縮めたとき)
    self->volt_saturated = (ff_scale < 0.999f) || (pi_scale < 0.999f);
    self->volt_pi_saturated = (pi_scale < 0.999f);
    self->volt_traction_limited = self->volt_saturated;
    OmniDrive_Send(self, mv, 2);  // command: 2 (Voltage)
    return;
  }
  for (int i = 0; i < 4; i++) self->cmd_voltage[i] = 0.0f;

  // 3. 出力整形・最大角速度クランプ
  int16_t m[4];
  for (int i = 0; i < 4; i++) {
    float v_clamped = Constrain(target_wheel_angular[i], -100.0f, 100.0f);
    int16_t raw_m = (int16_t)(v_clamped * 100.0f);

    // MAFは常に更新しておき (切替直後に古い値が出ないように)、S字制限が無効な場合のみ使う
    int16_t maf_m = (int16_t)MAF_Update(&self->maf[i], raw_m);
    m[i] = self->tcs.config.enable_s_curve ? raw_m : maf_m;
  }

  OmniDrive_Send(self, m, 1);  // command: 1 (Drive)
}

// 出力を電圧制御にするか速度モードにするかと、それに合わせたTCSの設定をまとめて切り替える。
//  - 電圧制御: TCSの介入 (accel_gain による加速度上限の引き下げ・対地速度への引き戻し) は使わない
//    (輪ごとのトルク上限がトラクション制御を担う)。S字の上限 (加速度・ジャーク) は volt_tune の値
//    (実行中に書き換えられる。既定値は VOLT_MODE_MAX_ACCEL など)
//  - 速度モード: 従来どおり (TCS_ENABLE, TCS_MAX_ACCEL, TCS_MAX_JERK)
// 切り替わったときは速度PIの積分を0に戻す
void OmniDrive_SetControlMode(OmniDrive* self, bool use_voltage_control) {
  if (use_voltage_control != self->use_voltage_control) {
    for (int k = 0; k < 3; k++) self->vel_fb_integral[k] = 0.0f;
    self->volt_saturated = false;
    self->volt_pi_saturated = false;
  }
  self->use_voltage_control = use_voltage_control;
  TCSConfig* cfg = &self->tcs.config;
  cfg->enable_tcs = use_voltage_control ? false : (TCS_ENABLE != 0);
  if (use_voltage_control) {
    cfg->max_accel = volt_tune.max_accel;
    cfg->max_ang_accel = volt_tune.max_ang_accel;
    cfg->max_jerk = volt_tune.max_jerk;
    cfg->max_ang_jerk = volt_tune.max_ang_jerk;
  } else {
    cfg->max_accel = TCS_MAX_ACCEL;
    cfg->max_ang_accel = TCS_MAX_ANG_ACCEL;
    cfg->max_jerk = TCS_MAX_JERK;
    cfg->max_ang_jerk = TCS_MAX_ANG_JERK;
  }
}

void OmniDrive_SetFree(OmniDrive* self) {
  TCS_Reset(&self->tcs);
  for (int i = 0; i < 4; i++) {
    self->target_wheel_angular[i] = 0.0f;
    self->cmd_voltage[i] = 0.0f;
  }
  for (int k = 0; k < 3; k++) self->vel_fb_integral[k] = 0.0f;
  self->volt_saturated = false;
  self->volt_pi_saturated = false;
  self->volt_traction_limited = false;
  int16_t m[4] = {0, 0, 0, 0};
  OmniDrive_Send(self, m, 0);  // command: 0 (Free)
}

// 電圧モード (command 2): 各輪の印加電圧 [V] を送る。WheelUnit側で ±MAX_AMP_VOLT(5V) にクランプされ、
// 電源電圧での正規化もWheelUnit側で行われる。0V は空転ではなく短絡ブレーキになる。
// ※ WheelUnitのホイールロック検知は +5.0V ちょうどが1秒続くと出力を切るので、上限張り付きに注意
void OmniDrive_SetVoltage(OmniDrive* self, const float volt[4]) {
  int16_t m[4];
  for (int i = 0; i < 4; i++) {
    self->cmd_voltage[i] = Constrain(volt[i], -5.0f, 5.0f);
    m[i] = (int16_t)(self->cmd_voltage[i] * 100.0f);
    self->target_wheel_angular[i] = 0.0f;
  }
  OmniDrive_Send(self, m, 2);  // command: 2 (Voltage)
}

void OmniDrive_Send(OmniDrive* self, int16_t* m, uint8_t command) {
  static Timer timer = {0};

  // 1ms 経過するまで送信しない
  if (Timer_ReadMs(&timer) < 1) return;

  // 前回のDMA送信が完了していない場合は送信できないため、タイマーを進めず
  // 次ループで再送を試みる
  if (self->serials[2]->huart->gState == HAL_UART_STATE_BUSY_TX) return;
  Timer_Reset(&timer);

#if OMNI_TX_AVOID_HEADER_BYTE
  // 【応急処置】データ中にヘッダと同じ 0xAA を出さない。WheelUnit の受信処理は 0xAA をヘッダとして
  // 同期するだけでチェックサムが無いため、1byte取りこぼした後にデータ中の 0xAA へ同期すると、同じ値が
  // 送られ続ける間は毎フレーム捨て続け、2秒の受信タイムアウトで mode 0 (短絡ブレーキ) に落ちていた
  // (実測: 電圧 +1.70V = 0x00AA を送り続けた区間で、1〜2輪が約0.75秒停止)。
  // 下位バイトが 0xAA の値は1LSB (0.01V / 0.01rad/s) ずらす。上位バイトが 0xAA になる値は使わない範囲。
  // WheelUnit側で対策されたら parammeter.h の OMNI_TX_AVOID_HEADER_BYTE を 0 にして外す
  for (int i = 0; i < 4; i++) {
    if ((m[i] & 0xFF) == 0xAA) m[i] = (int16_t)(m[i] + 1);
  }
#endif

  static uint8_t send_data[11];
  send_data[0] = 0xAA;
  send_data[1] = command;
  send_data[2] = (uint8_t)((m[0] >> 8) & 0xFF);
  send_data[3] = (uint8_t)(m[0] & 0xFF);
  send_data[4] = (uint8_t)((m[1] >> 8) & 0xFF);
  send_data[5] = (uint8_t)(m[1] & 0xFF);
  send_data[6] = (uint8_t)((m[2] >> 8) & 0xFF);
  send_data[7] = (uint8_t)(m[2] & 0xFF);
  send_data[8] = (uint8_t)((m[3] >> 8) & 0xFF);
  send_data[9] = (uint8_t)(m[3] & 0xFF);
  send_data[10] = 0xFF;

  Serial_Write(self->serials[2], send_data, 11);
}

// 受信した車輪角速度を受理するか判定する。前回値から OMNI_WHEEL_MAX_JUMP_RADPS を超えて
// 跳んだ値は保留し、次のフレームも保留値の近くなら本物の変化として受理する
// (ヘッダ/フッタ判定だけではデータ中の0xFF/0xAAでフレームがずれた誤値を受理しうるため)
static bool OmniDrive_AcceptWheelVel(float prev, float value, float* pending,
                                     bool* has_pending) {
  if (fabsf(value - prev) <= OMNI_WHEEL_MAX_JUMP_RADPS) {
    *has_pending = false;
    return true;
  }
  if (*has_pending && fabsf(value - *pending) <= OMNI_WHEEL_MAX_JUMP_RADPS) {
    *has_pending = false;
    return true;
  }
  *pending = value;
  *has_pending = true;
  return false;
}

void OmniDrive_Recv(OmniDrive* self) {
  static uint8_t recv_data[4][3];
  static uint8_t index[4] = {0};
  static float pending[4];
  static bool has_pending[4] = {false};

  for (int i = 0; i < 4; i++) {
    while (Serial_Available(self->serials[i])) {
      uint8_t recv_byte = Serial_Read(self->serials[i]);

      if (index[i] == 0) {
        if (recv_byte == 0xFF) {
          index[i]++;
        } else {
          index[i] = 0;
        }
      } else if (index[i] == 4) {
        if (recv_byte == 0xAA) {
          self->emg = recv_data[i][0] & 0x01;
          self->ready = (recv_data[i][0] >> 1) & 0x01;
          self->wheel_status[i] = recv_data[i][0];
          self->wheel_frame_count[i]++;
          self->wheel_last_frame_tick[i] = HAL_GetTick();
          self->wheel_rx_stalled[i] = false;
          float vel = (int16_t)((recv_data[i][1] << 8) | recv_data[i][2]) * 0.01f;
          if (OmniDrive_AcceptWheelVel(self->vel_wheel_angular[i], vel, &pending[i],
                                       &has_pending[i])) {
            self->vel_wheel_angular[i] = vel;
          }
        }
        index[i] = 0;
      } else {
        recv_data[i][index[i] - 1] = recv_byte;
        index[i]++;
      }
    }
  }

  OmniDrive_MonitorRx(self);
}

// 受信監視: 最後の正常フレームから OMNI_RX_TIMEOUT_MS 以上たった輪は受信を再開する。
// 実機では、MainBoard の動作中に WheelUnit が起動 (24Vの投入・瞬断) すると、
// 受信エラーのコールバックも呼ばれないまま4輪の受信が止まり、車輪速度が固まった。
// 原因の特定用に、途絶えるたびに最初の1回だけ USART/DMA の状態を出力する
// (WheelUnitが電源オフの間は OMNI_RX_TIMEOUT_MS ごとに再開を繰り返す)
static void OmniDrive_MonitorRx(OmniDrive* self) {
  uint32_t now = HAL_GetTick();
  if (!self->rx_monitor_started) {
    // 起動処理 (IMUキャリブレーション等) の時間を途絶と誤判定しないよう、初回に時刻を揃える
    for (int i = 0; i < 4; i++) self->wheel_last_frame_tick[i] = now;
    self->rx_monitor_started = true;
    return;
  }

  for (int i = 0; i < 4; i++) {
    if (now - self->wheel_last_frame_tick[i] < OMNI_RX_TIMEOUT_MS) continue;

    UART_HandleTypeDef* huart = self->serials[i]->huart;
    if (!self->wheel_rx_stalled[i]) {
      self->wheel_rx_stalled[i] = true;
      printf("# rx_stall wheel=%d t=%lu SR=0x%04lX CR3=0x%04lX NDTR=%lu EN=%lu RxState=0x%02X Err=0x%02lX\n",
             i, (unsigned long)now, (unsigned long)huart->Instance->SR,
             (unsigned long)huart->Instance->CR3, (unsigned long)huart->hdmarx->Instance->NDTR,
             (unsigned long)(huart->hdmarx->Instance->CR & DMA_SxCR_EN),
             (unsigned)huart->RxState, (unsigned long)huart->ErrorCode);
    }
    Serial_RestartRx(self->serials[i]);
    self->wheel_rx_restart_count[i]++;
    self->wheel_last_frame_tick[i] = now;  // 次の再開は OMNI_RX_TIMEOUT_MS 後
  }
}

void OmniDrive_GetVelF(const OmniDrive* self, float* vx, float* vy, float* omega) {
  float out[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 4; i++) {
    float v_wheel_linear = self->vel_wheel_angular[i] * ROBOT_WHEEL_RADIUS;
    for (int r = 0; r < 3; r++) out[r] += self->fk[r][i] * v_wheel_linear;
  }
  *vx = out[0];
  *vy = out[1];
  *omega = out[2];
}

void OmniDrive_GetVel(OmniDrive* self, int16_t* vel_x, int16_t* vel_y, int16_t* vel_angle) {
  float vx, vy, omega;
  OmniDrive_GetVelF(self, &vx, &vy, &omega);
  *vel_x = (int16_t)(vx * 1000.0f);
  *vel_y = (int16_t)(vy * 1000.0f);
  *vel_angle = (int16_t)(omega * 1000.0f);
}
