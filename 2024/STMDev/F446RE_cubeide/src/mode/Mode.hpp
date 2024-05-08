#ifndef __MODE__
#define __MODE__

#include <./unit/Robot.hpp>

class Mode {
  public:
    Mode(char letter, const char name[]) : modeLetter(letter) {
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

  protected:
    char modeName[24];
    char modeLetter;
};

#endif