#include "Mode.hpp"

class KickTestMode : public Mode {
  public:
    KickTestMode(char letter, const char name[], Robot *robotPtr) : Mode(letter, name, robotPtr) {}
    void before() override;
    void after() override;
    void loop() override;
    void encode(UIModeSwitch_t *_uiModeSwitchData) override;

  private:
    Timer timer;

};