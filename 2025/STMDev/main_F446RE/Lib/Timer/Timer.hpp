#ifndef __Timer__
#define __Timer__

#include "main.h"

#ifdef __cplusplus

extern "C" {

class Timer {
  public:
    // using DWT for timer watching
    Timer();
    
    void reset(){
        startTime = DWT->CYCCNT;
    }

    // HAL_RCC_GetSysClockFreq()はRCCレジスタから毎回64bit除算でクロックを計算し直すため重い
    // (メインループの時間待ちや1kHz割り込みで頻繁に呼ばれる)。HALが更新するSystemCoreClockを使う。
    // 注意: DWT->CYCCNTは180MHzで約23.8秒で一周するので、それより長い時間は測れない
    float read(){
        return (float)(DWT->CYCCNT - startTime) / SystemCoreClock;
    }

    uint32_t read_ms(){
        return (DWT->CYCCNT - startTime) / (SystemCoreClock / 1000);
    }

    uint32_t read_us(){
        return (DWT->CYCCNT - startTime) / (SystemCoreClock / 1000000);
    };

    uint32_t read_count(){
        return DWT->CYCCNT - startTime;
    }

    void set_ms(uint32_t ms);

  private:
    uint32_t startTime;
};

void wait_ns(uint32_t micros);
void wait_us(uint32_t micros);
void wait_ms(uint32_t micros);
}

#endif
#endif