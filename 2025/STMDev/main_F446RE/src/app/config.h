#ifndef __Parametors__
#define __Parametors__

#define PHOTOSENSOR_THRESHOLD 750
#define BATTERY_CUT_OFF 150

// ドリブラーボードのバージョン選択
// DRIBBLER_OLD: 2024年版基板
// (フォトセンサのみで保持判定。ドリブラーへは送信のみでフィードバックなし)
// DRIBBLER_NEW: 2025年版基板
// (ドリブラーからボール保持/検知のフィードバックをCAN受信)
#define DRIBBLER_OLD 0
#define DRIBBLER_NEW 1
#define DRIBBLER_VERSION DRIBBLER_NEW

// IMU(BNO055)の姿勢推定方式の選択
// IMU_FUSION_STM: BNO055はAMGモード(生データ)で使い、STM側で起動時ジャイロバイアス補正+Madgwick(地磁気なし)
//                 (2026年機体のLSM6DSO32と同じ方式。yawは起動時基準の相対角、反時計回りが正)
// IMU_FUSION_BNO_IMUPLUS: BNO055内部のフュージョン(IMUPLUSモード: 加速度+ジャイロ、地磁気なし)
//                 (yawは起動時基準の相対角、BNO055の仕様で時計回りが正)
#define IMU_FUSION_STM 0
#define IMU_FUSION_BNO_IMUPLUS 1
#define IMU_FUSION IMU_FUSION_STM

#define IMU_UPDATE_INTERVAL_MS 5      // BNO055の読み出し周期[ms] (AMGのジャイロ出力は約100Hz)
#define IMU_DETECT_TIMEOUT_MS 1000    // 起動後この時間[ms]まではBNO055の応答を待つ (電源投入から応答まで約650ms)
#define IMU_CALIB_SAMPLE_COUNT 200    // ジャイロバイアス測定のサンプル数(10ms間隔で約2秒)
#define IMU_MAX_DT_S 0.05f            // Madgwick積分1ステップのdt上限[s](呼び出し遅延時に過大な回転を積分しないため)
#define IMU_MADGWICK_BETA 0.1f        // Madgwickフィルタのゲイン(2026年機体と同じ)
// 静止中のジャイロバイアス追従 (IMU_FUSION_STMのみ)
#define IMU_STILL_GYRO_DPS 0.5f       // 全軸の角速度(バイアス補正後)がこれ未満なら静止候補
#define IMU_STILL_TIME_MS 500         // FF速度0かつ静止候補がこの時間続いたら静止とみなす
#define IMU_BIAS_TRACK_ALPHA 0.002f   // 静止中、1サンプル(5ms)ごとにバイアスを測定値へ寄せる割合(時定数 約2.5秒)

// Piとの通信途絶の判定時間[ms]。これ以上正しいパケットが届かなければ信号なし扱いにして停止する
#define RAS_SIGNAL_TIMEOUT_MS 200

// ウォッチドッグ(IWDG)のタイムアウト[ms]。メインループが止まる(HardFault等)とこの時間でリセットされる
// (LSIの誤差で実際は約0.7〜1.9倍になる)
#define WATCHDOG_TIMEOUT_MS 200

// CAN Standard ID
// kickboard
#define PHOTOTHRESHOLD 0x09       // 15
#define CHARGE_START 0x10         // 16
#define DISCHARGE_START 0x11      // 17
#define KICK_STRAIGHT 0x12        // 18
#define KICK_CHIP 0x13            // 19
#define KICKER_BOARD_PACKET 0x123 // 291

// motor driver board
#define MD_SEND 0x1AA // 426
#define SEND_EMG 0x00 // 0
#define MD0_RECV 0x200
#define MD1_RECV 0x201
#define MD2_RECV 0x202
#define MD3_RECV 0x203

// dribbler board
#if DRIBBLER_VERSION == DRIBBLER_OLD
#define DRIBBLE 0x14 // 20
#else
#define DRIBBLE_SEND 0xF4
#define DRIBBLE_RECV 0xF5
#endif

#endif