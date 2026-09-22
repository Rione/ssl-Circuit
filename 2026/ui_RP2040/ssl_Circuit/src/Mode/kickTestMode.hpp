#include "Mode.hpp"

class KickTestMode : public Mode {
  public:
    KickTestMode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) : Mode(letter, name, uiPtr, mediaPtr) {
    }

    void displaySet() override;

    void determine() override;

  private:
    void kickUI();
    
};