#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <Arduino.h>
#include <TFT_eSPI.h>

#define ARR_SIZE 76800

class DISPLAY_DEVICE {
   public:
    TFT_eSPI* tftPtr;
    TFT_eSprite* spritePtr;

    DISPLAY_DEVICE(TFT_eSPI* tftPtr, TFT_eSprite* spritePtr);

    void init(void);
    void setSPIClockFast(void);

    void publish(int x = 0, int y = 0);
    void setBackgroundImage(const uint16_t *imagePtr);

    void createSprite(int x = 240, int y = 320);

    void displayLight(bool light);

   private:
    const int backlightPin = 0;
};

#endif