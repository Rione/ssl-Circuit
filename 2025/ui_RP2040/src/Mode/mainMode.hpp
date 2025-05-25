#include "Mode.hpp"

class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) : Mode(letter, name, uiPtr, mediaPtr) {
    }

    void displaySet() override;

    void determine() override;

  private:
    void mainUI();
};