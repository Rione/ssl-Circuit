#ifndef __PARAMMETER_H_
#define __PARAMMETER_H_

#include <stdint.h>

// 機体パラメータ
#define ROBOT_WHEEL_RADIUS 0.03f        // 車輪半径[m]
#define ROBOT_WHEEL_BASE_RADIUS 0.075f  // 車輪基底径[m]
#define ROBOT_RADIUS 0.089f             // ロボット半径[m]

extern const int16_t ROBOT_MOTOR_DEGREE[4];  // モーターの取り付け角度[deg]

// 制御
#define ROBOT_CONTROL_LOOP_DT_US 1000  // 制御ループ周期[us]
#define ROBOT_MAX_VEL 3.0f             // 最大並進速度[m/s]
#define ROBOT_MAX_ANG_VEL 10.0f        // 最大角速度[rad/s]

// トラクションコントロール (TCS) パラメータ
#define TCS_ENABLE 1                   // TCS有効化フラグ (1: 有効, 0: 無効)
#define TCS_ENABLE_S_CURVE 1           // S字/ジャーク制限加減速有効化 (1: 有効, 0: 無効)
#define TCS_MAX_ACCEL 18.0f            // 最大並進加速度 [m/s^2] (摩擦限界を考慮: 約1.8G)
#define TCS_MAX_JERK 250.0f            // 最大並進ジャーク [m/s^3] (俊敏な応答とスパイク低減の両立)
#define TCS_MAX_ANG_ACCEL 60.0f        // 最大角加速度 [rad/s^2]
#define TCS_MAX_ANG_JERK 600.0f        // 最大角ジャーク [rad/s^3]
#define TCS_GEOM_K 1.158456f           // 4輪幾何拘束係数: sqrt(2)*sin(55deg)
#define TCS_GEOM_SLIP_THRESH 15.0f     // 幾何残差スリップ判定閾値 [rad/s] (定常公差残差による誤検知防止)
#define TCS_ROT_SLIP_THRESH 4.5f       // 旋回ジャイロ残差スリップ判定閾値 [rad/s]
// 加速度残差 (a_odom - a_imu の指令加速度方向成分) の判定閾値 [m/s^2]
// (全輪が比例して空転する直進加速時のスリップは幾何残差/旋回残差に現れないため、これで検出する)
#define TCS_ACCEL_SLIP_THRESH 4.0f
// 加速度残差で判定するのは指令加速度がこれ以上のときだけ [m/s^2]
// (巡航中は車輪速度PIDのリップルで a_odom が±数m/s²揺れ、常時誤検知していたため)
#define TCS_ACCEL_DETECT_MIN 1.0f
#define TCS_VEL_SLIP_EXIT 0.10f        // スリップ解除条件: |v_odom - v_ground| がこれ未満 [m/s]
#define TCS_SLIP_HOLD_MS 50            // または、スリップ要因が消えてからこの時間経過で解除 [ms]
#define TCS_SLIP_VEL_MARGIN 0.15f      // スリップ中に許す 指令速度 - 対地速度 の上限 [m/s]
#define TCS_GROUND_VEL_KC 30.0f        // 対地速度推定の相補フィルタゲイン(グリップ中にodomへ寄せる強さ) [1/s]
#define TCS_ACCEL_LPF_HZ 15.0f         // odom加速度/IMU加速度に掛ける一次LPFのカットオフ [Hz] (位相を揃える)
#define TCS_MAX_INERTIAL_MS 300        // スリップ中にIMU積分のみで対地速度を推定する最大時間 [ms] (ドリフト防止)
#define TCS_MIN_GAIN 0.25f             // スリップ時の加速度上限比率の下限 (max_accel に対する比)
#define TCS_SLIP_GAIN_CUT 0.6f         // スリップ突入1回あたりの加速度上限比率の最大カット (×0.6 = 最大4割減)
#define TCS_RECOVERY_RATE 6.0f         // グリップ回復後の加速度上限比率の復帰速度 [1/s]
#define TCS_DEADZONE_SPEED_MPS 0.10f   // 低速デッドゾーン [m/s] (指令・実測とも未満ならスリップ判定しない)

// WheelUnitから受信した車輪角速度の1フレームあたりの最大変化 [rad/s]
// これを超える跳びは1フレームだけ保留し、次フレームで確認できなければ破棄する。
// 物理的な車輪角加速度は最大でも約600rad/s² (18m/s² / 0.03m) で1フレームあたり数rad/s。
// 実測ではフレームずれと思われる130rad/s超の単発異常値があり、スリップ誤検知の原因になっていた
#define OMNI_WHEEL_MAX_JUMP_RADPS 20.0f
// WheelUnitからの受信がこの時間 [ms] 途絶えたら受信を再開する (正常時は約0.2msごとに届く)
#define OMNI_RX_TIMEOUT_MS 20U

// 【応急処置】WheelUnitへの送信データにヘッダと同じ 0xAA を出さない (下位バイトが 0xAA の値を1LSBずらす)。
// WheelUnitの受信処理にチェックサムが無く、データ中の 0xAA に同期すると同じ値を送り続ける間は受信できず、
// 2秒の受信タイムアウトで mode 0 (短絡ブレーキ) に落ちていたため (引き継ぎ文書 5.4)。
// WheelUnit側でチェックサム等の対策が入ったら 0 にしてよい。1 の間は起動時に "# WORKAROUND:" を出力する
#define OMNI_TX_AVOID_HEADER_BYTE 1

// ※ 以下の WHEEL_VOLT_KA_*_BODY, VEL_FB_*, VOLT_TRACTION_LIMIT_V, VOLT_MODE_MAX_*、および電圧制御のときの
//    TCS_MAX_JERK / TCS_MAX_ANG_JERK は「既定値」。実際に使うのは volt_tune (src/config/volt_tune.h) の値で、
//    自動チューニングや ST-Link で実行中に書き換えられる (安全範囲は volt_tune.c)
// 電圧制御のフィードフォワード (src/control/wheel_voltage.c)
// 加減速に使う電圧。機体の加速度を力の配分 (OmniDrive.force_alloc) で4輪に配るときの係数。
//  並進 [V/(m/s^2)]: 前後の効き方を従来 (車輪の角加速度あたり 0.015V、= 0.015/r) と同じにした値。
//    3.0m/sテストの加速区間で機体1m/s²あたり1輪約0.45V要っていた。左右は配分により約1.3〜1.6倍になる
//  回転 [V/(rad/s^2)]: 従来の車輪の角加速度あたり 0.007V (= 0.007×R/r)。並進と同じ係数だと
//    回り始めに1輪約2Vかかり、指令の約1.8倍の速さで回った
#define WHEEL_VOLT_KA_LIN_BODY 0.5f
// 左右 (機体の y) の加速度ぶんの FF 係数 [V/(m/s^2)]。既定は WHEEL_VOLT_KA_LIN_BODY と同じ。
// 左右は、実際に出る加速度が前後の約 0.67 倍で (ランプ試験、HANDOFF_AUTOTUNE.md 10.10)、FF が左右の必要な電圧を
// 足りなく見積もっている可能性がある。FF 試験 (ramp_test の -Speeds ff) で較正して、この値を決める
#define WHEEL_VOLT_KA_LAT_BODY 0.5f
// 1: 起動時に、フラッシュに保存された調整値 (自動最適化 optimizer.c が見つけた ka_lin・ka_lat) を読み込んで、既定値にする。
// 保存された値は試合でも使われる。無効にするか、ST-Link から消す (autotune_start.ps1 -ClearSaved) ときは注意
#define AUTOTUNE_LOAD_SAVED 1
#define WHEEL_VOLT_KA_ANG_BODY 0.0175f
// 床の上の負荷分の電圧 (輪ごとの値は wheel_voltage.c) を足すか。浮かせて試すときは 0 にする
#define WHEEL_VOLT_USE_LOAD_FF 1
#define WHEEL_VOLT_MAX 4.9f        // 印加電圧の上限 [V] (WheelUnitは +5.0V ちょうどが1秒続くと出力を切る)

// 電圧制御の機体速度フィードバック (PI)。機体の3自由度で誤差を計算し、逆運動学で4輪に配る
// (4輪が互いに逆らう成分は出ない)。並進はオドメトリ、回転はジャイロの速度を使う
#define VEL_FB_KP_LIN 1.5f   // 並進 P [V/(m/s)] (無負荷で約1(m/s)/V)
#define VEL_FB_KI_LIN 5.0f   // 並進 I [V/(m/s·s)]
#define VEL_FB_KP_ANG 0.2f   // 回転 P [V/(rad/s)] (4輪に同じ電圧を足すと約13.6(rad/s)/V で回る)
// 回転 I [V/(rad/s·s)]。0.4 では旋回の最後の3°を詰めるのに約2秒かかった (小さな指令だと摩擦を超えない)
#define VEL_FB_KI_ANG 1.5f
#define VEL_FB_I_MAX_V 2.0f  // 積分項の上限 [V] (ワインドアップ防止)

// 電圧制御のトルク上限 (トラクション制御): 各輪の電圧を、その輪の実際の回転数で転がり続ける電圧
// ± VOLT_TRACTION_LIMIT_V に抑える (＝モータトルクの上限。超えるときは4輪を同じ比率で縮め、
// 向き・横ずれを直すPIの分を優先して残す)。電圧制御ではTCSの accel_gain による介入は使わない
// (TCSテストで enable_tcs=false)。
// 1.3V では車輪はほぼ滑らず (実測と対地速度の回転数の差は中央値 −2%)、加減速は約2.5m/s²しか出なかった
// ので 2.0V に上げて様子を見た (3.0m/s テスト、引き継ぎ文書 5.8)。力の配分を入れた動作パターンの
// テストで 2.4V→3.2V と上げ、3.2V ではスリップが50〜87%まで増えた (セグメントによっては動いている
// 時間の大半が空転)。速さと安定感のバランスを取り、間の 2.8V に落ち着けた
#define VOLT_TRACTION_LIMIT_V 2.8f
// 電圧制御のときのS字加減速の加速度上限 [m/s^2] (速度モードの TCS_MAX_ACCEL=18 は実機で出ない)。
// 目標が実機より先へ行きすぎないよう、出せる加速度に近い値にする。トルク上限 2.8V に合わせる
#define VOLT_MODE_MAX_ACCEL 5.0f
// 電圧制御のときのS字の角加速度上限 [rad/s^2] (速度モードの TCS_MAX_ANG_ACCEL=60 では回りすぎた)
#define VOLT_MODE_MAX_ANG_ACCEL 38.0f

// 試合の経路 (Rock5A からの指令、Robot_SendOmniDrive) の出力を電圧制御にするか (1: 電圧, 0: 速度モード)。
// 床のテスト (前後・左右・斜め・旋回、動作パターンのテスト) で確かめてから 1 にした。
// Robot_SendOmniDrive は OmniDrive_SetControlMode と IMU 付きで OmniDrive_SetVelEx を呼んでおり、
// 動作パターンのテストと同じ経路 (electric制御・トルク上限・PI) を通る。
// ⚠ 実際に Rock5A (SPI) から指令を受けて走らせる確認はまだ行っていない。低速から確かめること
#define ROBOT_USE_VOLTAGE_CONTROL 1

// 1: Rock5A からの指令 (走行・キック・ドリブル) を受け付けない。自動チューニングが完成するまでの間、
//    Rock5A を付けたままでも ST-Link から指示したテスト (auto_tune.c) が取り消されないようにする。
//    Rock5A の緊急停止は、信号を受信していて emergency_stop=1 のときだけ、テストを取り消して止まる安全のために見続ける
//    (信号が来ていない間は emergency_stop が常に1なので、それは見ない)。
//    ⚠ 1 の間は Rock5A から機体を動かせない。試合・Rock5A での走行確認の前に 0 に戻すこと
#define AUTOTUNE_IGNORE_ROCK_COMMANDS 1

// TCSテスト (LocalController_TestTCSAcceleration) の出力を電圧制御にするか (1: 電圧, 0: 速度モード)
#define TEST_TCS_USE_VOLTAGE_CONTROL 1
#define TEST_TCS_SPEED_MMPS 3000   // TCSテストの目標速度 [mm/s]
#define TEST_TCS_DISTANCE_M 2.0f   // TCSテストの往復距離 [m] (前後に約0.5mの空きが要る)
// TCSテストで減速を始める位置を決める想定の減速度 [m/s^2]。残りの距離 ≤ v²/(2×これ) で反転を指令する
// (区間の端で反転すると、3m/s では止まるまでに約1m行き過ぎる。4.0 ではまだ約0.6m行き過ぎた。
//  トルク上限2.0Vでの実際の減速は約2.4〜2.9m/s²だったので2.5にした。2.8Vへの引き上げに合わせて4.0に上げる。
//  実際の減速が足りなければ安全停止 (はみ出し0.5m) が掛かるので、床でもう一度確かめること)
#define TEST_TCS_BRAKE_DECEL 4.0f
#define TEST_TCS_ABORT_HEADING_RAD 0.785f  // TCSテストの安全停止: ヘディングのずれ [rad] (45°)
// 動作パターンのテストの速度上限 [m/s] (2.5 で右へ3mの区間が上限に届いた。トルク上限引き上げに合わせて
// 3.5 に上げる。ROBOT_MAX_VEL=3.0 を超えるのはテスト専用の値のため)
#define TEST_PATTERN_SPEED_MPS 3.5f
// 動作パターンのテストの移動量の倍率 (1.0 で前後・左右1m)。安全停止の範囲も同じ倍率で広がる。
// 1.5 のとき、スタート位置から前に約2.0m、後ろに約0.5m、左右に約2.0mの空きが要る
#define TEST_PATTERN_SCALE 1.5f
#define TEST_PATTERN_ANG_VEL_RADPS 11.0f  // 動作パターンのテストの旋回の上限 [rad/s] (12 で上限近くまで出ていた)
#define TEST_PATTERN_ANG_BRAKE 25.0f      // 動作パターンのテストで向きの目標へ減速する想定の角減速度 [rad/s^2]
#define TEST_TCS_ABORT_OVERRUN_M 0.5f      // TCSテストの安全停止: 往復区間からのはみ出し [m] (前後の空きに合わせる)

#define ROBOT_KICK_INTERVAL_MS ((uint32_t)1000)  // キック間隔[ms]
#define ROBOT_KICKER_SIGNAL_INTERVAL_MS \
  ((uint32_t)100)  // チャージ/放電信号の最小送信周期[ms]

#define ROBOT_STOP_DISCHARGE_SPEED_MMPS \
  ((int16_t)100)  // 停止時にこの速度[mm/s]を超えていたら強制放電

// IMU
#define IMU_MADGWICK_BETA 0.1f  // Madgwickフィルタのゲイン(加速度補正の強さ)

// IMU取付方向: センサ-y = 機体前方(+x), センサ+x = 機体左方(+y) (z軸回りの回転なのでジャイロzはそのまま)
#define IMU_TO_ROBOT_AX(sx, sy) (-(sy))  // 機体x方向加速度
#define IMU_TO_ROBOT_AY(sx, sy) (sx)     // 機体y方向加速度
#define IMU_GRAVITY_MPS2 9.80665f        // [g] -> [m/s^2]

// IMUのバイアス (ジャイロ3軸・加速度xy) の決め方:
//   1: 起動時 (Robot_Initialize) に測定する。測定中は機体を静止させること (約3秒、動いていれば
//      最大5回測り直す)。静止と判定できたらフラッシュに保存する (保存時に1〜2秒止まる)。
//      静止と判定できなければ、保存済みの値があればそれを使う。
//   0: フラッシュに保存済みの値を使う (起動時に測定しないので、電源投入後すぐ動かしてよい)。
//      保存が無い場合 (未保存・フラッシュ全消去後) は起動時に測定する。
// 運用: 1 で書き込み → 静止させて起動し "saved to flash" を確認 → 0 に戻して書き込む
#define IMU_CALIBRATE_ON_BOOT 0

#endif  // __PARAMMETER_H_
