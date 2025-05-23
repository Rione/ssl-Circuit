#include "Mode.hpp"

class Home : public Mode {
  public:
    Home(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) : Mode(letter, name, uiPtr, mediaPtr) {
    }

    void displaySet() override;

    void determine() override;    

  private:
    void homeUI();
    
};