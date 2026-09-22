#ifndef __KickerBoard__
#define __KickerBoard__

#include "CAN.hpp"
#include "config.h"
#include "Timer.hpp"

class KickerBoard {
  public:
    KickerBoard(CANBus *canBus);

    void init();
    
    // -- kickのタイプ、パワー、ダイレクトを指定してキックする
    // type : 0: straight, 1: chip
    // power: 0~100
    // doDirect: 0 : normal, 1: doDirect
    void kick(uint8_t type, uint8_t power, uint8_t doDirect = 0);

    // -- doDirectのリセット
    // type : 0: straight, 1: chip
    void resetDoDirect(uint8_t type);

    // -- charge, dischargeのコントロール
    // state: 0: discharge, 1: doCharge  
    void chargeControl(uint8_t state);

    void minusCapValEstimate(uint8_t val) {
        capValEstimate -= val;
        if (capValEstimate < 0) {
            capValEstimate = 0;
        }
    }

    inline __attribute__((always_inline)) uint8_t getCapValEstimate() {
        return capValEstimate;
    }

    inline __attribute__((always_inline)) void setCapValEstimate(uint8_t val) {
        capValEstimate = val;
    }

  private:
    CANBus *_canBus;
    uint8_t capValEstimate; // 充電量の推定値
};

#endif