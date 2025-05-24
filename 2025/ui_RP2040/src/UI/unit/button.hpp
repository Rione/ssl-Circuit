#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <Arduino.h>

#include "UI/unit/display.h"
#include "UI/unit/touchscreen.h"

#include "UI/image/image.h"
#include "UI/image/top.h"
#include "UI/image/kick.h"
#include "UI/image/main_img.h"
#include "UI/image/home_img.h"

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

  private:
    DISPLAY_DEVICE *display;
    TOUCHSCREEN *touch;
    int x; // ボタンのx座標
    int y; // ボタンのy座標
    int w; // ボタンの幅
    int h; // ボタンの高さ
    const uint16_t *redimage;   
    const uint16_t *whiteimage; 

    int time = 0; // ボタンが押された時間
};

#endif