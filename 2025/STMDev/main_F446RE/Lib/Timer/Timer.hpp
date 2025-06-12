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

    float read(){
        return (float)(DWT->CYCCNT - startTime) / HAL_RCC_GetSysClockFreq();
    }

    uint32_t read_ms(){
        return (uint32_t)((float)(DWT->CYCCNT - startTime) / HAL_RCC_GetSysClockFreq() * 1000);
    }

    uint32_t read_us(){
        return (uint32_t)((float)(DWT->CYCCNT - startTime) / HAL_RCC_GetSysClockFreq() * 1000000);
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