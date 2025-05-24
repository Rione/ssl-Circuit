#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <Arduino.h>
#include <TFT_eSPI.h>

#define ARR_SIZE 76800

class DISPLAY_DEVICE {
  public:
    DISPLAY_DEVICE(TFT_eSPI *tftPtr, TFT_eSprite *spritePtr);

    void init(void);
    void setSPIClockFast(void);

    void publish(int x = 0, int y = 0);
    void setBackgroundImage(const uint16_t *imagePtr);
    void setParttImage(int w, int h, const uint16_t *imagePtr);

    void createSprite(int x = 320, int y = 240);

    void displayLight(bool light);

    static const int backlightPin = 0;

  private:
    TFT_eSPI *tftPtr;
    TFT_eSprite *spritePtr;
    
};

#endif