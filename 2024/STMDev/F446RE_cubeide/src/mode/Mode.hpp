#ifndef __MODE__
#define __MODE__

#include <./unit/Robot.hpp>

class Mode {
  public:
    Mode(char letter, const char name[], Robot *robotPtr) : modeLetter(letter), robot(robotPtr) {
      strcpy(modeName, name);
    }

    char *getModeName() {
      return modeName;
    }

    char getModeLetter() {
      return modeLetter;
    }

    virtual void init() {

    };
    virtual void before() {

    };
    virtual void loop() {

    };
    virtual void after() {

    };

    virtual void encode(UIModeSwitch_t *_uiModeSwitchData) {
      
    };
    
  protected:
    char modeName[24];
    char modeLetter;
    // uint8_t data[5];
    Robot *robot;
    // UIModeSwitch_t *uiModeSwitchData;
};

#endif