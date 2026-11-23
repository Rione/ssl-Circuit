// src/mode/ModePolicy の試験。PC の g++ で動かす (ファームには入らない)。
//
//   cd test && make
//
// 試合用のビルド (bench = false) では、どの入力でもカメラモードと UI の走行試験に入らないことを確かめる。
// 台上用のビルド (bench = true) は今までの振る舞いのまま。

#include <stdio.h>

#include "ModePolicy.hpp"

using mode_policy::Drive;
using mode_policy::RunMode;

static int failures = 0;

#define CHECK(cond)                                                    \
      do {                                                             \
            if (!(cond)) {                                             \
                  printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
                  failures++;                                          \
            }                                                          \
      } while (0)

static const bool kMatch = false;
static const bool kBench = true;

// IMU スイッチを押していても (読みが 0)、試合用のビルドはメインモードで起動する
static void testMatchBuildAlwaysStartsInMainMode() {
      CHECK(mode_policy::selectRunMode(kMatch, true) == RunMode::kMain);
      CHECK(mode_policy::selectRunMode(kMatch, false) == RunMode::kMain);
}

// 台上用のビルドは、IMU スイッチを押して起動したときだけカメラモード
static void testBenchBuildEntersCameraModeOnlyWithSwitchPressed() {
      CHECK(mode_policy::selectRunMode(kBench, true) == RunMode::kMain);
      CHECK(mode_policy::selectRunMode(kBench, false) == RunMode::kCamera);
}

// 試合用のビルドは UI のドリブラ試験とモーター試験を受け付けない。キック・充電・放電は受け付ける
static void testMatchBuildRejectsUiDriveTests() {
      CHECK(!mode_policy::acceptsUiCommand(kMatch, mode_policy::kUiCommandDribbler));
      CHECK(!mode_policy::acceptsUiCommand(kMatch, mode_policy::kUiCommandMotor));
      CHECK(mode_policy::acceptsUiCommand(kMatch, mode_policy::kUiCommandKick));
      CHECK(mode_policy::acceptsUiCommand(kMatch, mode_policy::kUiCommandChipKick));
      CHECK(mode_policy::acceptsUiCommand(kMatch, mode_policy::kUiCommandDischarge));
}

static void testBenchBuildAcceptsAllUiCommands() {
      for (int command = mode_policy::kUiCommandNone; command <= mode_policy::kUiCommandDischarge; command++) {
            CHECK(mode_policy::acceptsUiCommand(kBench, (uint8_t)command));
      }
}

// 試合用のビルドは、UI の試験の印が立っていても、非常停止なら止め、信号が無ければ止め、
// 信号があって非常停止でないときだけ Pi の指令に従う
static void testMatchBuildNeverRunsUiTest() {
      for (int ui = 0; ui <= 1; ui++) {
            CHECK(mode_policy::arbitrate(kMatch, ui, true, true) == Drive::kStop);
            CHECK(mode_policy::arbitrate(kMatch, ui, true, false) == Drive::kStop);
            CHECK(mode_policy::arbitrate(kMatch, ui, false, false) == Drive::kStop);
            CHECK(mode_policy::arbitrate(kMatch, ui, false, true) == Drive::kFollowPi);
      }
}

// 台上用のビルドは、UI の試験中は非常停止と信号より先に試験を動かす (今までの振る舞い)
static void testBenchBuildRunsUiTestBeforeEmergencyStop() {
      CHECK(mode_policy::arbitrate(kBench, true, true, true) == Drive::kUiTest);
      CHECK(mode_policy::arbitrate(kBench, true, false, false) == Drive::kUiTest);
      CHECK(mode_policy::arbitrate(kBench, false, true, true) == Drive::kStop);
      CHECK(mode_policy::arbitrate(kBench, false, false, false) == Drive::kStop);
      CHECK(mode_policy::arbitrate(kBench, false, false, true) == Drive::kFollowPi);
}

int main() {
      testMatchBuildAlwaysStartsInMainMode();
      testBenchBuildEntersCameraModeOnlyWithSwitchPressed();
      testMatchBuildRejectsUiDriveTests();
      testBenchBuildAcceptsAllUiCommands();
      testMatchBuildNeverRunsUiTest();
      testBenchBuildRunsUiTestBeforeEmergencyStop();
      if (failures) {
            printf("%d failure(s)\n", failures);
            return 1;
      }
      printf("ok\n");
      return 0;
}
