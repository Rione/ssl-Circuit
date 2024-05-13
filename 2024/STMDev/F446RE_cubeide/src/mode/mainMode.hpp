#include "Mode.hpp"

class MainMode : public Mode {

  public:
    MainMode(char letter, const char name[], Robot *robotPtr) : Mode(letter, name, robotPtr) {}

    void before() {
    }

    void loop() {
        robot->led1 = !robot->led1;
        HAL_Delay(500);
        printf("Main Mode\n");
    }

    void after() {
    }

  private:
};
