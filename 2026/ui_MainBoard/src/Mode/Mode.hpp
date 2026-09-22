#ifndef _Mode_
#define _Mode_

#include "AppManager/ui_kit.hpp"
#include "Media/MediaExecutor.hpp"

class Mode {
  public:
    Mode(char letter, const char name[], UiKit *uiPtr, MediaExecutor *mediaPtr) : modeLetter(letter), ui(uiPtr), media(mediaPtr) {
        strcpy(modeName, name);
    }

    virtual void displaySet() {
    };

    virtual void determine() {
    };

  protected:
    char modeName[24];
    char modeLetter;
    UiKit *ui;
    MediaExecutor *media;
};

#endif