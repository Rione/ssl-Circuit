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

    bool isRobotStop;

    inline void checkRobotStop() {
        static int velZeroCount = 0;
        if (robot->info.velX.vel == 0 && robot->info.velY.vel == 0 && robot->info.velAngler.vel == 0) {
            if (velZeroCount < 1000) {
                velZeroCount++;
            } else {
                velZeroCount = 1000;
                isRobotStop = true;
            }
        } else {
            velZeroCount = 0;
            isRobotStop = false;
        }
    }

    inline void boosterManager() {
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
        }
    }
};
