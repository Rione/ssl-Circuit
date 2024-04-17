#ifndef __Timer__
#define __Timer__

#include "main.h"

#ifdef __cplusplus

extern "C" {

class Timer {
  public:
    // using DWT for timer watching
    Timer();

    void reset();

    float read();

    uint32_t read_ms();

    uint32_t read_us();

    uint32_t read_count();

  private:
    uint32_t startTime;
};

void wait_ns(uint32_t micros);
void wait_us(uint32_t micros);
void wait_ms(uint32_t micros);
}

#endif
#endif