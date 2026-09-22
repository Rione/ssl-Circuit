#include "Timer.hpp"

Timer::Timer() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Timer::reset() {
    startTime = DWT->CYCCNT;
}

float Timer::read() {
    return (float)(DWT->CYCCNT - startTime) / HAL_RCC_GetSysClockFreq();
}

uint32_t Timer::read_ms() {

    return (uint32_t)((float)(DWT->CYCCNT - startTime) / HAL_RCC_GetSysClockFreq() * 1000);
}

uint32_t Timer::read_us() {
    return (uint32_t)((float)(DWT->CYCCNT - startTime) / HAL_RCC_GetSysClockFreq() * 1000000);
}

uint32_t Timer::read_count() {
    return DWT->CYCCNT - startTime;
}

void Timer::set_ms(uint32_t ms) {
    uint32_t current_count = DWT->CYCCNT;
    uint32_t sys_clk_freq = HAL_RCC_GetSysClockFreq();
    uint32_t count_offset = (ms * (sys_clk_freq / 1000));
    startTime = current_count - count_offset;
}

void wait_ns(uint32_t micros) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t requiredTicks = micros * (SystemCoreClock / 1000000000);

    while ((DWT->CYCCNT - startTick) < requiredTicks) {
        // Wait until the required number of ticks has elapsed
    }
}

void wait_us(uint32_t micros) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t requiredTicks = micros * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - startTick) < requiredTicks) {
        // Wait until the required number of ticks has elapsed
    }
}

void wait_ms(uint32_t micros) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t requiredTicks = micros * (SystemCoreClock / 1000);

    while ((DWT->CYCCNT - startTick) < requiredTicks) {
        // Wait until the required number of ticks has elapsed
    }
}