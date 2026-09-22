#include "Wire.h"
#include <cstring>
#include "i2c.h"    // 追加: hi2c1 の extern 宣言を取得

TwoWire Wire(&hi2c1);

TwoWire::TwoWire(I2C_HandleTypeDef *hi2c)
{
    m_i2c = hi2c;
    txIndex = 0;
    rxIndex = 0;
    rxLength = 0;
    transmitting = 0;
    txAddress = 0;
    memset(txBuffer, 0, sizeof(txBuffer));   // clear buffers
    memset(rxBuffer, 0, sizeof(rxBuffer));
}

uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(m_i2c, (uint16_t)txAddress, txBuffer, txIndex, 100);

    txIndex = 0;
    transmitting = 0;

    if (ret == HAL_OK) return 0;
    if (ret == HAL_BUSY) return 2;   // map to Arduino BUSY if desired
    if (ret == HAL_TIMEOUT) return 5;
    return 4;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
    if (quantity > BUFFER_LENGTH) quantity = BUFFER_LENGTH;

    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(m_i2c, (uint16_t)(address << 1), rxBuffer, quantity, 100);

    if (ret == HAL_OK) {
        rxIndex = 0;
        rxLength = quantity;
        return quantity;
    } else {
        rxIndex = 0;
        rxLength = 0;
        return 0;
    }
}

void TwoWire::begin()
{
    // HAL の初期化は CubeMX に任せる想定
}

void TwoWire::setClock(uint32_t frequency)
{
    m_i2c->Init.ClockSpeed = frequency;
    HAL_I2C_Init(m_i2c);
}

void TwoWire::beginTransmission(uint8_t address)
{
    transmitting = 1;
    txAddress = address << 1;
    txIndex = 0;
}

size_t TwoWire::write(uint8_t data)
{
    if (txIndex >= BUFFER_LENGTH) return 0;
    txBuffer[txIndex++] = data;
    return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
    if (txIndex + quantity >= BUFFER_LENGTH) return 0;
    memcpy(&txBuffer[txIndex], data, quantity);
    txIndex += quantity;
    return quantity;
}

int TwoWire::available()
{
    return rxLength - rxIndex;
}

int TwoWire::read()
{
    if (rxIndex >= rxLength) return -1;
    return rxBuffer[rxIndex++];
}

int TwoWire::peek()
{
    if (rxIndex >= rxLength) return -1;
    return rxBuffer[rxIndex];
}

void TwoWire::flush()
{
    rxIndex = 0;
    rxLength = 0;
    txIndex = 0;
}
