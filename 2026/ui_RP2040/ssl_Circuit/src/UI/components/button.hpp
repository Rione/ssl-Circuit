#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <Arduino.h>

#include "display.h"
#include "touchscreen.h"

#include "UI/images/image.h"
#include "UI/images/top.h"
#include "UI/images/kick.h"
#include "UI/images/main_img.h"
#include "UI/images/home_img.h"

// UIボタンに関するクラス
class BUTTON {
  public:
    BUTTON(DISPLAY_DEVICE *display, TOUCHSCREEN *touch, int x, int y, int w, int h, const uint16_t *redimage, const uint16_t *whiteimage) {
        this->display = display;
        this->touch = touch;
        this->x = x;
        this->y = y;
        this->w = w;
        this->h = h;
        this->redimage = redimage;
        this->whiteimage = whiteimage;
    }

    bool determine();

    void setWhiteImg();

    bool buttonDisable(); //ボタンの連続押しを防ぐための関数 500ms、1->ボタン無効化

  private:
    DISPLAY_DEVICE *display;
    TOUCHSCREEN *touch;
    int x; // ボタンのx座標
    int y; // ボタンのy座標
    int w; // ボタンの幅
    int h; // ボタンの高さ
    const uint16_t *redimage;   
    const uint16_t *whiteimage; 

    uint32_t pressTime = 0; // ボタンが押された時間
    uint32_t releaseTime = 0; // ボタンが離された時間
};


class TEXT_BUTTON {
  public:
    TEXT_BUTTON(DISPLAY_DEVICE *display, TOUCHSCREEN *touch, int x, int y, int w, int h, const char* text, uint16_t bg, uint16_t fg);
    void draw(bool state);
    void updateState();
    bool determine();
  private:
    DISPLAY_DEVICE *display;
    TOUCHSCREEN *touch;
    int x, y, w, h;
    const char* text;
    uint16_t bg, fg;
    bool isPressed = false;
    uint32_t pressTime = 0;
};


#endif
