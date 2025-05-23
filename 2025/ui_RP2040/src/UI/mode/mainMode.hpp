#include "Mode.hpp"

class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) : Mode(letter, name, uiPtr), media(mediaPtr) {
    }

    void displaySet(UiKit *_ui) override;

    void determine(UiKit *_ui) override;

    void mainUI(UiKit *_ui);

  private:
    bool isTouched_state = false;
    bool isTouched_kick = false;
    int isTouchedTime = 0;
    int TouchedInterval;
    MediaExecutor *media;
};