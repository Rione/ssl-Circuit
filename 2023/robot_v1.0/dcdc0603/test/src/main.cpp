#include "mbed.h"
#include "modeRun.h"

DigitalOut chageStart(PA_4);
InterruptIn chageDone(PA_3);

bool kickable = false;

void chageDoneIRQ(){
    kickable = true;
}

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

    chageDone.fall(&chageDoneIRQ);
}

int main(void) {
    setup();
    while (1) {
        while (mode == 'M') {
            pc.printf("send K to start chaging\n");
            wait(1);
        }
        chageStart = true;
        while (kickable == false){
            pc.printf("chaging...\n");
            wait_ms(100);
        }
        kickable = false;
        
        chageStart = false;
        while (mode != 'M') {
            pc.printf("chage may done... Press M to kick\n");
            wait(1);
        }
        kicker[STRAIGHT_KICKER].setPower(1.0); // power:0.0~1.0
        kicker[STRAIGHT_KICKER].Kick();
    }
}