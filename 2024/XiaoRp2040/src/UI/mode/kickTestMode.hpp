#include "Mode.hpp"

class KickTestMode : public Mode {
  public:
    KickTestMode(char letter, const char name[], UiKit *uiPtr) : Mode(letter, name, uiPtr) {
    }

    void displaySet(UiKit *_ui) override;

    void determine(UiKit *_ui) override;

  private:
    void kickUI();


    
};