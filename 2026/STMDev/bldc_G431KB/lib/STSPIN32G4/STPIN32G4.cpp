#include "STSPIN32G4.hpp"

STSPIN32G4::STSPIN32G4(I2C_HandleTypeDef *hi2c)
    : wake(WAKE_GPIO_Port, WAKE_Pin),
      ready(READY_GPIO_Port, READY_Pin),
      nFault(NFAULT_GPIO_Port, NFAULT_Pin) {
    _hi2c = hi2c;
    wake = true;
}

STSPIN32G4::~STSPIN32G4() {}

bool STSPIN32G4::i2cWrite(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
    uint8_t i2c_data[1] = {data};
    if (HAL_I2C_Mem_Write(_hi2c, deviceAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, i2c_data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    return true;
}

void STSPIN32G4::init() {
    reset();
    HAL_Delay(10);
    setBuckConverterVoltage(8);
    setNfaultStatus();
    HAL_Delay(10);
    clearRegister();
}

void STSPIN32G4::writeRegister(uint8_t registerValue) {
    uint8_t data[1] = {registerValue};
    HAL_I2C_Master_Transmit(_hi2c, STSPING4_CONTROLER_ADDR, data, 1, 100); // data is the start pointer of our array
}

void STSPIN32G4::setBuckConverterVoltage(uint8_t voltage) {
    uint8_t value = 0b00000000;
    switch (voltage) {
    case 8:
        value = 0b00000000;
        break;
    case 10:
        value = 0b00000001;
        break;
    case 12:
        value = 0b00000010;
        break;
    case 15:
        value = 0b00000011;
        break;
    default:
        value = 0b00000000;
        printf("Invalid voltage value. Set to 8V\n");
        break;
    }
    bool i2c_status = false;
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b11110000);
    printf("unlock!! READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, POWMNG_REG, value);
    printf("Writereg READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b00000000);
    printf("lock!!   READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
}

void STSPIN32G4::setNfaultStatus() {
    /*
    NFAULT  Address 0x08  [[Protected]]
    bit7                0:
    bit6                1:
    bit5                1:
    bit4                1:
    bit3                1:
    bit2 [VDS_P_FLT]    1 :Signaling of the VDS protection triggering: Enabled by default, 0 to disable
    bit1 [THSD _FLT]    1 :Signaling of the thermal shutdown status: Enabled by default, 0 to disable
    bit0 [VCC_UVLO_FLT] 1 :Signaling of the VCC UVLO status: Enabled by default, 0 to disable
    */

    uint8_t value = 0b01111111;
    // value &= ~(1 << 2); // VDS_P_FLT 0: disable
    // value &= ~(1 << 1); // THSD_FLT 0: disable
    // value &= ~(1 << 0); // VCC_UVLO_FLT 0: disable

    bool i2c_status = false;
    // unlock
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b11110000);
    printf("unlock!! READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, NFAULT_REG, value);
    printf("Writereg READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b00000000);
    printf("lock!!   READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
}

bool STSPIN32G4::clearRegister() {
    bool i2c_status = false;
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, CLEAR__REG, 0b11111111);
    return i2c_status;
}

void STSPIN32G4::reset() {
    bool i2c_status = false;
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b11110000);
    printf("unlock!! READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, RESET_REG, 0xFF);
    printf("Writereg READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
    i2c_status = i2cWrite(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b00000000);
    printf("lock!!   READY:%d NFAULT:%d I2C_success:%d\n", ready.read(), nFault.read(), i2c_status);
    HAL_Delay(10);
}

void STSPIN32G4::readStatus() {
    uint8_t data[1];
    uint8_t addr = STATUS_REG;
    HAL_I2C_Master_Transmit(&hi2c3, STSPING4_CONTROLER_ADDR << 1, &addr, 1, 100);
    HAL_I2C_Master_Receive(&hi2c3, STSPING4_CONTROLER_ADDR << 1, data, 1, 100);
    printf("LOCK:%d RESET:%d VDS_P:%d THSD:%d VCC_UVLO:%d\n", (data[0] >> 7) & 0x01, (data[0] >> 3) & 0x01, (data[0] >> 2) & 0x01, (data[0] >> 1) & 0x01, (data[0] >> 0) & 0x01);
}