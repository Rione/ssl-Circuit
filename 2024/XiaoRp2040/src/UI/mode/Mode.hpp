#ifndef _Mode_
#define _Mode_

#include "UI/ui_kit.hpp"

class Mode {
  public:
    Mode(char letter, const char name[]) : modeLetter(letter){
        strcpy(modeName, name);
    }

    virtual void displaySet(){

    };

    virtual void determine(){

    };
  
  private:
    char modeName[24];
    char modeLetter;    

};


#endif