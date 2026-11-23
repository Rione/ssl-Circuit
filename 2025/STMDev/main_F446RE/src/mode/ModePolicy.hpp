#ifndef __ModePolicy__
#define __ModePolicy__

// 動作モードの選び方と、足回り・ドリブラをこの周期に何の指令で動かすかの決まり。
// bench は config.h の BENCH_MODES (台上の試験用の動作を入れたビルドか)。試合用のビルドでは false。
// HAL に依存しない (test/mode_policy_test.cpp を PC の g++ で動かすため)。

#include <stdint.h>

namespace mode_policy {

enum class RunMode : uint8_t {
  kMain = 0,    // Pi (RAVEN) の指令で動く
  kCamera = 1,  // 機体のカメラのボールへ寄る。Pi の速度の指令は見ない (台上用のビルドだけ)
};

// 起動時の IMU スイッチ (基板の SW5「IMU」。プルアップなので離すと 1、押すと 0) の読みから動作モードを決める。
// 試合用のビルドでは、スイッチがどうであってもメインモードにする。
constexpr RunMode selectRunMode(bool bench, bool imuSwitchLevel) {
  return (bench && !imuSwitchLevel) ? RunMode::kCamera : RunMode::kMain;
}

// UI (タッチパネル、2025/ui_RP2040) から届く試験の指令の番号
constexpr uint8_t kUiCommandNone = 0;
constexpr uint8_t kUiCommandKick = 1;
constexpr uint8_t kUiCommandChipKick = 2;
constexpr uint8_t kUiCommandDribbler = 3;
constexpr uint8_t kUiCommandMotor = 4;
constexpr uint8_t kUiCommandDischarge = 5;

// UI の試験の指令を受け付けるか。ドリブラ試験とモーター試験は非常停止と Pi の指令より先に動くので、
// 台上用のビルドだけで受け付ける。キック・充電・放電は今までどおり受け付ける。
constexpr bool acceptsUiCommand(bool bench, uint8_t command) {
  return bench || (command != kUiCommandDribbler && command != kUiCommandMotor);
}

enum class Drive : uint8_t {
  kStop,      // 止める (stopRobot)
  kFollowPi,  // Pi の指令に従う
  kUiTest,    // UI の走行試験 (台上用のビルドだけ)
};

// この周期に足回りとドリブラを何で動かすか。
// 台上用のビルドだけは、UI の走行試験を非常停止と信号より先に動かす (今までの振る舞い)。
constexpr Drive arbitrate(bool bench, bool uiTestActive, bool emergencyStop, bool signalReceived) {
  if (bench && uiTestActive) {
    return Drive::kUiTest;
  }
  if (emergencyStop) {
    return Drive::kStop;
  }
  if (signalReceived) {
    return Drive::kFollowPi;
  }
  return Drive::kStop;
}

}  // namespace mode_policy

#endif
