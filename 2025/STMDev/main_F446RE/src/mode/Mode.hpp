#ifndef __MODE__
#define __MODE__

#include <./unit/RobotSequence.hpp>

class Mode {
  public:
    Mode(char letter, const char name[], Robot *robotPtr) : modeLetter(letter), robot(robotPtr) {
        strcpy(modeName, name);
    }

    char *getModeName() {
        return modeName;
    }

    char getModeLetter() {
        return modeLetter;
    }

    virtual void init() {

    };
    virtual void before() {

    };
    virtual void loop() {

    };
    virtual void after() {

    };

  protected:
    // 前回の締め切りからperiodUs後まで待つ(固定周期)。ある周期が長引いても次の周期で取り戻すので、
    // 平均周期がずれない。1周期以上遅れた場合は取り戻そうとせず、今を基準に仕切り直す。
    // 締め切りが1周期より先にある場合(初回)も仕切り直す。DWT->CYCCNTは電源を切らないリセットでは
    // 0に戻らないので、loopDeadline=0のままだと最初の待ちが最大約12秒になってしまう。
    void waitNextPeriod(uint32_t periodUs) {
        const uint32_t period = periodUs * (SystemCoreClock / 1000000);
        loopDeadline += period;
        const int32_t late = (int32_t)(DWT->CYCCNT - loopDeadline);
        if (late > (int32_t)period || late < -(int32_t)period) {
            loopDeadline = DWT->CYCCNT;
            return;
        }
        while ((int32_t)(loopDeadline - DWT->CYCCNT) > 0);
    }

    char modeName[24];
    uint32_t loopDeadline = 0;
    char modeLetter;
    Robot *robot;
};

#endif