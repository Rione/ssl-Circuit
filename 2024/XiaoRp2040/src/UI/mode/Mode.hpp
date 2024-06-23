#ifndef _Mode_
#define _Mode_

#include "UI/ui_kit.hpp"

class Mode {
  public:
    Mode(char letter, const char name[], UiKit *uiPtr) : modeLetter(letter), ui(uiPtr){
        strcpy(modeName, name);
    }

    virtual void displaySet(UiKit *_ui){

    };

    virtual void determine(UiKit *_ui){

    };
  
  private:
    char modeName[24];
    char modeLetter;  
    UiKit *ui;

};


#endif