#include "Mode.hpp"

class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[]) : Mode(letter, name) {
    }

    void displaySet() override;

    void determine() override;
    
};