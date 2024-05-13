#include "Mode.hpp"

class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[], Robot *robotPtr) : Mode(letter, name, robotPtr) {}

    void before() {
    }

    void loop() {
        robot->rasRecvSerial();
        robot->motorDriver.setVelocityFF(robot->info.velX.vel,
                                         robot->info.velY.vel,
                                         (float)(robot->info.velAngler.vel) * 0.001);

        printf("velX: %d, velY: %d, velAngler: %d\n", robot->info.velX.vel, robot->info.velY.vel, robot->info.velAngler.vel);
    }

    void after() {
    }
};
