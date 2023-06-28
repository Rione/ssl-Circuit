#include "mbed.h"
#include "modeRun.h"

DigitalOut chageStart(PA_4);
InterruptIn chageDone(PA_3);

void setup() {
    swDrible.mode(PullUp);
    swKicker.mode(PullUp);
    // dribler.write(0);
    dribbler.turnOff();
    pc.baud(2000000);
    imu.init();
    MD.setVelocityZero();
    initModeRun();
    imu.setZero();
    timer.start();
    pidDt.start();
    tickCalcIMU.attach_us(&attitudeControl, 10000);
    setPIDGain();
}

int main(void) {
    setup();
    while (1) {
        while (mode == 'M') {
            pc.printf("send K to start chaging\n");
            wait(1);
        }
        chageStart = true;
        for (size_t i = 0; i < 30; i++) {
            pc.printf("chaging...\n");
            wait_ms(100);
        }
        chageStart = false;
        while (mode != 'M') {
            pc.printf("chage may done... Press M to kick\n");
            wait(1);
        }
    }
}