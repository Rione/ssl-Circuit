#include "mbed.h"
#include "LGKicker.h"

LGKicker::LGKicker(PinName KickerPin)
    : Kicker(KickerPin), KickerIsRedey(), Kicker_timeout(), timer() {
    Kicker = 0;
    timer.start();
    timer.reset();
    Kicker.period_us(10000); // 50000
    setPower(1.0);
}

bool LGKicker::Kick(void) {
    if (timer.read_ms() > 2000) {
        if (enabled == 0) {
            KickerIsRedey.attach_us(callback(this, &LGKicker::flipOn), 2000);
            enabled = 1;
        }
        return 1;
    } else {
        Kicker = 0;
        return 0;
    }
}

void LGKicker::flipOn(void) {
    Kicker = power;
    Kicker_timeout.attach_us(callback(this, &LGKicker::flip), 30000);
}
void LGKicker::flip(void) {
    Kicker = 0;
    timer.reset();
    enabled = 0;
}

void LGKicker::setPower(float p) {
    power = (p < 0.0 ? 0.0 : (p > 1.0 ? 1.0 : p));
}

void LGKicker::discharge(void) {
    Kicker = 0;
    setPower(0.15);
    for (size_t i = 0; i < 250; i++) {
        KickerIsRedey.attach_us(callback(this, &LGKicker::flipOn), 2000);
        wait(0.05);
    }
}