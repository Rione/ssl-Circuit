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

    void checkRobotStop();
    void boosterManager();
};
