#include "touchscreen.h"

void TOUCHSCREEN::read() {
    digitalWrite(csPin, LOW);
    isTouched_1ms = ptr->touched();
    isTouchedPrev = isTouched;

    // 誤タップ防止
    // 1msごとのタッチ判定（周期的には4ms）
    // タッチカウントの最大値を超えたらタッチ判定
    // 離されてタッチカウントの最小値を超えたらタッチ判定を解除
    if (isTouched_1ms && !isTouched) {
        if (touchCount_1ms < 1) {
            Time_touchStart = millis();
            touchCount_1ms = 1;
        } else if (millis() - Time_touchStart > 1 && touchCount_1ms < countMax) {
            touchCount_1ms++;
            Time_touchStart = millis();
        } else if (millis() - Time_touchStart > 1 && touchCount_1ms >= countMax) {
            isTouched = true;
            touchCount_1ms = 0;
        }
    }else if (!isTouched_1ms && isTouched) {
        if (touchCount_1ms > -1) {
            Time_touchStart = millis();
            touchCount_1ms = -1;
        } else if (millis() - Time_touchStart > 1 && touchCount_1ms > countMin) {
            touchCount_1ms--;
            Time_touchStart = millis();
        } else if (millis() - Time_touchStart > 1 && touchCount_1ms <= countMin) {
            isTouched = false;
            touchCount_1ms = 0;
        }
    }

    if (isTouched) {
        TS_Point p = ptr->getPoint();
        raw.x = p.x;
        raw.y = p.y;

        point.x =
            constrain(map(p.x, upperLeft.x, bottomRight.x, 0, 320), 0, 320);
        point.y =
            constrain(map(p.y, upperLeft.y, bottomRight.y, 0, 240), 0, 240);
    }

    digitalWrite(csPin, HIGH);
}