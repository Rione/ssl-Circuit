#include "SPI.h"

SPIClass SPI(nullptr);

SPIClass::SPIClass(SPI_HandleTypeDef *hspi)
{
    m_hspi = hspi;
}

void SPIClass::begin()
{
    applySettings();
}

void SPIClass::end()
{
    HAL_SPI_DeInit(m_hspi);
}

void SPIClass::beginTransaction(SPISettings settings)
{
    currentSettings = settings;
    applySettings();
}

void SPIClass::endTransaction()
{
    // STM32 HAL では特に処理なし
}

void SPIClass::applySettings()
{
    m_hspi->Init.Mode = SPI_MODE_MASTER;

    // Clock polarity/phase
    switch(currentSettings.dataMode) {
        case SPI_MODE0:
            m_hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
            m_hspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
            break;
        case SPI_MODE1:
            m_hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
            m_hspi->Init.CLKPhase    = SPI_PHASE_2EDGE;
            break;
        case SPI_MODE2:
            m_hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
            m_hspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
            break;
        case SPI_MODE3:
            m_hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
            m_hspi->Init.CLKPhase    = SPI_PHASE_2EDGE;
            break;
    }

    // Bit order
    m_hspi->Init.FirstBit = (currentSettings.bitOrder == MSBFIRST)
                                ? SPI_FIRSTBIT_MSB
                                : SPI_FIRSTBIT_LSB;

    // Clock speed → BaudRatePrescaler へ変換
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();   // SPI1 の例（SPI2/3 は PCLK1）
    uint32_t presc = pclk / currentSettings.clockFreq;

    if (presc <= 2)       m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    else if (presc <= 4)  m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    else if (presc <= 8)  m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    else if (presc <= 16) m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    else if (presc <= 32) m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    else if (presc <= 64) m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    else if (presc <= 128)m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    else                  m_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;

    HAL_SPI_Init(m_hspi);
}

uint8_t SPIClass::transfer(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(m_hspi, &data, &rx, 1, 100);
    return rx;
}

uint16_t SPIClass::transfer16(uint16_t data)
{
    uint16_t rx;
    HAL_SPI_TransmitReceive(m_hspi, (uint8_t*)&data, (uint8_t*)&rx, 2, 100);
    return rx;
}

void SPIClass::transfer(void *buf, size_t count)
{
    HAL_SPI_TransmitReceive(m_hspi, (uint8_t*)buf, (uint8_t*)buf, count, 100);
}

void SPIClass::transfer(const void *tx, void *rx, size_t count)
{
    HAL_SPI_TransmitReceive(m_hspi, (uint8_t*)tx, (uint8_t*)rx, count, 100);
}

void SPIClass::setBitOrder(BitOrder order)
{
    currentSettings.bitOrder = order;
    applySettings();
}

void SPIClass::setDataMode(SPIMode mode)
{
    currentSettings.dataMode = mode;
    applySettings();
}

void SPIClass::setClockDivider(uint8_t divider)
{
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    currentSettings.clockFreq = pclk / divider;
    applySettings();
}
