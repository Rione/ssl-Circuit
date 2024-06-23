#include "Mode.hpp"

class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[], UiKit *uiPtr) : Mode(letter, name, uiPtr) {
    }

    void displaySet() override;

    void determine(UiKit *_ui) override;
    
  private:
    bool changeFlag = true;
};