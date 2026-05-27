#pragma once
#include "stm32f4xx_hal.h"   // ← MCUに合わせて変更（例：stm32f4xx_hal.h）
#include <stdint.h>
#include <stddef.h>

#define BUFFER_LENGTH 32

class TwoWire {
public:
    TwoWire(I2C_HandleTypeDef *hi2c);

    void begin();
    void setClock(uint32_t frequency);

    void beginTransmission(uint8_t address);
    uint8_t endTransmission(uint8_t sendStop = 1);

    uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop = 1);

    size_t write(uint8_t data);
    size_t write(const uint8_t *data, size_t quantity);

    int available(void);
    int read(void);
    int peek(void);
    void flush(void);

private:
    I2C_HandleTypeDef *m_i2c;

    uint8_t txAddress;
    uint8_t txBuffer[BUFFER_LENGTH];
    uint8_t txIndex;

    uint8_t rxBuffer[BUFFER_LENGTH];
    uint8_t rxIndex;
    uint8_t rxLength;

    uint8_t transmitting;
};

extern TwoWire Wire;
