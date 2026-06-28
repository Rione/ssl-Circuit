#ifndef _TEST_MODE_
#define _TEST_MODE_

#include "Mode.hpp"
#include "UI/components/button.hpp"

class TestMode : public Mode {
  public:
    TestMode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr);

    void displaySet() override;
    void determine() override;

  private:
    void drawUI();

    TEXT_BUTTON *btnDribbler;
    TEXT_BUTTON *btnKick;
    TEXT_BUTTON *btnChipKick;
    TEXT_BUTTON *btnMotorTest;
    TEXT_BUTTON *btnCharge;
    TEXT_BUTTON *btnDischarge;
};

#endif
