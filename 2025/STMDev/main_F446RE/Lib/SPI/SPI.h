#pragma once
#include "stm32f4xx_hal.h"   // ← 使用する MCU に合わせて変更

#include <stdint.h>
#include <stddef.h>

#define SPI_HAS_TRANSACTION 1

// Arduino 互換
#define MSBFIRST 0
#define LSBFIRST 1

typedef enum {
    SPI_MODE0 = 0,
    SPI_MODE1,
    SPI_MODE2,
    SPI_MODE3
} SPIMode;

typedef uint8_t BitOrder;

class SPISettings {
public:
    constexpr SPISettings(uint32_t clock, BitOrder bitOrder, SPIMode dataMode)
        : clockFreq(clock), bitOrder(bitOrder), dataMode(dataMode)
    {}

    constexpr SPISettings()
        : clockFreq(1000000), bitOrder(MSBFIRST), dataMode(SPI_MODE0)
    {}

    uint32_t clockFreq;
    BitOrder bitOrder;
    SPIMode dataMode;
};

class SPIClass {
public:
    SPIClass(SPI_HandleTypeDef *hspi);

    void begin();
    void end();

    void beginTransaction(SPISettings settings);
    void endTransaction();

    uint8_t transfer(uint8_t data);
    uint16_t transfer16(uint16_t data);
    void transfer(void *buf, size_t count);
    void transfer(const void *tx, void *rx, size_t count);

    void setBitOrder(BitOrder order);
    void setDataMode(SPIMode mode);
    void setClockDivider(uint8_t divider);  // 互換のため用意（中身は HAL 用に変換）

private:
    SPI_HandleTypeDef *m_hspi;
    SPISettings currentSettings;

    void applySettings();
};

extern SPIClass SPI;
