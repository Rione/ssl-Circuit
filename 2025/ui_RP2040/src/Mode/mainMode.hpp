#include "Mode.hpp"

class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) : Mode(letter, name, uiPtr, mediaPtr) {
    }

    void displaySet() override;

    void determine() override;

  private:
    bool isTouched_state = false;
    bool isTouched_kick = false;
    int isTouchedTime = 0;
    int TouchedInterval;

    void mainUI();
};