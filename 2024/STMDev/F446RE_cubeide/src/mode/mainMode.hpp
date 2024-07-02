#include "Mode.hpp"
#include "Average.h"
#include "Median.h"
class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[], Robot *robotPtr) : Mode(letter, name, robotPtr) {}
    void before() override;
    void after() override;
    void loop() override;

  private:
    float meanVelXBuf[15];
    float meanVelYBuf[15];
    float meanVelAngBuf[15];

    Average<float> meanVelX = Average(meanVelXBuf, 15);
    Average<float> meanVelY = Average(meanVelYBuf, 15);
    Average<float> meanVelAngler = Average(meanVelAngBuf, 15);

    Timer timer;

    inline void boosterManager() {
        static Timer timer;
        // ロボットの状態に関わらず常に行う処理
        if (robot->swDischarge.isRelease()) {
            if (robot->swDischarge.readPressedTime() > 1600) {
                robot->discharge();
                robot->led2 = false;
                printf("discharge start\n");
            } else if (robot->swDischarge.readPressedTime() > 800) {
                robot->chargeStart();
                robot->led2 = true;
                printf("charge start\n");
            } else if (robot->swDischarge.readPressedTime() > 200) {
                robot->kickStraight(100);
                printf("kick\n");
            }
        } else {
            if (timer.read_ms() > 100) { // 100msごとに実行
                if (robot->info.status.doCharge == true) {
                    // Piから充電しろと言われている。
                    if (robot->info.isKickerChargeMode == false) {
                        // KickerBoardから充電していないとの情報を得ている。噛み合っていない
                        robot->chargeStart();
                    }
                } else {
                    // Piから放電しろと来ている
                    if (robot->info.isKickerChargeMode == true) {
                        // KickerBoardから充電しているとの情報を得ている。噛み合っていない
                        robot->discharge();
                    }
                }
            }
        }
    }
};
