#include "Mode.hpp"

class Home : public Mode {
  public:
    Home(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) : Mode(letter, name, uiPtr), media(mediaPtr) {
    }

    void displaySet(UiKit *_ui) override;

    void determine(UiKit *_ui) override;

    void homeUI(UiKit *_ui);

  private:
    MediaExecutor *media;
};