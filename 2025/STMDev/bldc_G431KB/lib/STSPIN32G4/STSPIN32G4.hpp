#ifndef __STSPIN32G4__
#define __STSPIN32G4__
#ifdef __cplusplus

#include "main.h"
#include "i2c.h"
#include "DigitalInOut.hpp"
#include <stdio.h>

#define STSPING4_CONTROLER_ADDR 0b1000111
#define I2C_DATA_SIZE 8

#define POWMNG_REG 0x01
#define LOGIC_REG 0x02
#define READY_REG 0x07
#define NFAULT_REG 0x08
#define CLEAR__REG 0x09
#define STBY_REG 0x0A
#define LOCK_REG 0x0B
#define RESET_REG 0x0C
#define STATUS_REG 0x80

class STSPIN32G4 {
  private:
    /* data */
    unsigned char i2c_data[I2C_DATA_SIZE];
    int read_end;
    I2C_HandleTypeDef *_hi2c;
    DigitalOut wake;
    DigitalIn ready;
    DigitalIn nFault;

  public:
    STSPIN32G4(I2C_HandleTypeDef *hi2c);
    ~STSPIN32G4();
    void init();
    bool i2cWrite(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
    void writeRegister(uint8_t registerValue);
    void setBuckConverterVoltage(uint8_t voltage);
    void setNfaultStatus();
    bool clearRegister();
    void reset();
    void readStatus();
};

#endif
#endif