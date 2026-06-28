#include "button.hpp"

bool BUTTON::determine() {
    // タッチ判定
    // ボタンが離された時を検出する
    if (touch->isTouched) {
        if (touch->point.x > x && touch->point.x < x + w && touch->point.y > y && touch->point.y < y + h) {
            if(!touch->isTouchedPrev) display->setParttImage(x, y, w, h, redimage);
            pressTime = millis(); // ボタンが押された時間を記録
            return false; // ボタンが押された
        }
    } else if (!touch->isTouched && touch->isTouchedPrev) {
        if (touch->point.x > x && touch->point.x < x + w && touch->point.y > y && touch->point.y < y + h) {
            releaseTime = millis(); // ボタンが離された時間を記録
            return true; // ボタンが離された
        }
    }
    return false; // ボタンが押されていない
    
}

void BUTTON::setWhiteImg() {
    // ボタンが押されていない状態で500ms以上経過していたら白色に戻す
    // ただし、ボタンが押された時間が0の時は無視する
    if(pressTime != 0 && millis() - pressTime > 500) {
        // 500ms以上経過していたら白色に戻す
        display->setParttImage(x, y, w, h, whiteimage);
        pressTime = 0; // 時間をリセット
    }
}

bool BUTTON::buttonDisable() {
    // ボタンを無効化する
    // ボタンが押されてから500ms以内なら無効化
    if(millis() - releaseTime < 1000) return true; // ボタン無効化
    else return false; // ボタン有効化
}
    

TEXT_BUTTON::TEXT_BUTTON(DISPLAY_DEVICE *display, TOUCHSCREEN *touch, int x, int y, int w, int h, const char* text, uint16_t bg, uint16_t fg) {
    this->display = display;
    this->touch = touch;
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    this->text = text;
    this->bg = bg;
    this->fg = fg;
}

void TEXT_BUTTON::draw(bool state) {
    uint16_t drawBg = state ? display->sprite->color565(200, 200, 200) : bg;
    uint16_t borderCol = display->sprite->color565(200, 200, 200);
    
    display->sprite->fillRoundRect(x, y, w, h, 4, drawBg);
    display->sprite->drawRoundRect(x, y, w, h, 4, borderCol);
    display->sprite->setTextColor(fg);
    display->sprite->setTextDatum(ML_DATUM);
    display->sprite->setTextFont(2);
    display->sprite->drawString(text, x + 16, y + h / 2);
    display->sprite->setTextFont(1);
}

void TEXT_BUTTON::updateState() {
    if (isPressed && millis() - pressTime > 150) {
        isPressed = false;
        draw(false);
        display->publish(0, 0);
    }
}

bool TEXT_BUTTON::determine() {
    bool inBounds = touch->point.x > x && touch->point.x < x + w &&
                    touch->point.y > y && touch->point.y < y + h;

    if (touch->isTouched) {
        if (inBounds && !isPressed) {
            isPressed = true;
            pressTime = millis();
            draw(true);
            display->publish(0, 0);
        }
        return false;
    }

    if (isPressed) {
        isPressed = false;
        draw(false);
        display->publish(0, 0);
        return true;
    }
    return false;
}
