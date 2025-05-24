#include "button.hpp"

bool BUTTON::determine() {
    // タッチ判定
    // ボタンが離された時を検出する
    // 
    if (touch->isTouched && !touch->isTouchedPrev) {
        if (touch->point.x > x && touch->point.x < x + w && touch->point.y > y && touch->point.y < y + h) {
            display->setParttImage(w, h, redimage);
            display->publish(x, y);
            return false; // ボタンが押された
        }
    } else if (!touch->isTouched && touch->isTouchedPrev) {
        if (touch->point.x > x && touch->point.x < x + w && touch->point.y > y && touch->point.y < y + h) {
            time = millis();
            return true; // ボタンが離された
        }
    }
    return false; // ボタンが押されていない
    
}

void BUTTON::setWhiteImg() {
    // ボタンが押されていない状態で500ms以上経過していたら白色に戻す
    // ただし、ボタンが押された時間が0の時は無視する
    if(time != 0 && millis() - time > 500) {
        // 500ms以上経過していたら白色に戻す
        display->setParttImage(w, h, whiteimage);
        display->publish(x, y);
        time = 0; // 時間をリセット
    }
}