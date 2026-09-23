// UI 基板の設定値 (ピン・閾値・テスト上限など)
#pragma once

#include <stdint.h>

// ---- ピン (RP2040 GPIO 番号、回路図 UI シート参照) -------------------------
// LCD / タッチの SPI ピンは platformio.ini の build_flags (TFT_eSPI 設定) 側で定義
#define PIN_BACKLIGHT 0  // D6 LCD_LED (PWM で輝度制御)
#define PIN_BUZZER 7     // D5 UI_Audio
#define PIN_UART_TX 28   // D2 UI_TX (UI -> MainBoard)
#define PIN_UART_RX 29   // D3 UI_RX (MainBoard -> UI)

#define UART_BAUD 250000     // MainBoard UART4 と一致させる
#define UART_RX_FIFO 512     // 画面転送(~30ms)中に取りこぼさないよう大きめに確保
#define COMMAND_PERIOD_MS 20 // UI -> MainBoard コマンド送信周期
#define LINK_LOST_MS 500     // この時間テレメトリが来なければ NO LINK 表示

// ---- 画面 -------------------------------------------------------------------
#define SCREEN_ROTATION 1   // 1: 横向き。上下逆に実装されたら 3 にする
#define FRAME_PERIOD_MS 50  // 描画周期 (20fps)
#define BACKLIGHT_MAX 255   // 通常時の輝度 (0-255)

// 起動ロゴ
#define SPLASH_FADE_IN_MS 900
#define SPLASH_HOLD_MS 1200
#define SPLASH_FADE_OUT_MS 900

// ---- タッチ (XPT2046 生値 -> 画面座標) ----------------------------------------
// 2026/ui_MainBoard の較正値 (XPT2046_Touchscreen rotation3 相当) を TFT_eSPI の生値に換算したもの。
// ずれる場合は TOUCH_DEBUG を 1 にしてシリアルに出る生値から再設定する。
#define TOUCH_SWAP_XY 1   // 画面X <- 生Y, 画面Y <- 生X
#define TOUCH_INVERT_X 1  // 4095 - 生値
#define TOUCH_INVERT_Y 1
#define TOUCH_X_MIN 250
#define TOUCH_X_MAX 3700
#define TOUCH_Y_MIN 250
#define TOUCH_Y_MAX 3800
#define TOUCH_Z_THRESHOLD 350  // 押下判定の圧力閾値
#define TOUCH_DEBUG 0

// ---- ブザー -----------------------------------------------------------------
#define BUZZER_ENABLE 1
#define BUZZER_CLICK_HZ 2400
#define BUZZER_CLICK_MS 15

// ---- 表示閾値 ---------------------------------------------------------------
// 電源電圧の色分け [V]。使用バッテリーのセル数に合わせて調整すること
#define BATTERY_FULL_V 16.8f
#define BATTERY_EMPTY_V 13.2f
#define BATTERY_WARN_V 14.4f
#define BATTERY_LOW_V 13.6f

#define CAP_MAX_V 250.0f    // 昇圧電圧バーの最大値 [V]
#define CAP_READY_V 100.0f  // これ以上で「キック可能」表示 (MainBoard の LED1 閾値と同じ)

// ---- テスト -----------------------------------------------------------------
#define TEST_AUTO_LOCK_MS 60000  // 無操作でこの時間経過したら自動ロック

#define DRIBBLE_STEP_PCT 10
#define KICK_STEP_PCT 10
#define KICK_DEFAULT_PCT 30
#define KICK_MIN_INTERVAL_MS 1000  // MainBoard の ROBOT_KICK_INTERVAL_MS と揃える

#define WHEEL_TEST_MAX_RADPS 50.0f  // 目標角速度の上限 [rad/s] (MainBoard 側でも制限する)
#define WHEEL_TEST_STEP_RADPS 5.0f
#define WHEEL_TEST_DEFAULT_RADPS 10.0f
#define WHEEL_DISPLAY_MAX_RADPS 100.0f  // 速度バーのフルスケール (MainBoard の指令上限)
