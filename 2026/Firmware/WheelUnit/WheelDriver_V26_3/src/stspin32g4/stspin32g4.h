/* C implementation of STSPIN32G4 driver
 * Rewritten from original C++ implementation to plain C.
 */

#ifndef STSPIN32G4_H
#define STSPIN32G4_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "digitalinout.h"
#include "i2c.h"
#include "main.h"

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

void STSPIN32G4_Init(I2C_HandleTypeDef* hi2c);
void STSPIN32G4_Wake(void);
void STSPIN32G4_Configure(void);
bool STSPIN32G4_WaitReady(uint32_t timeout_ms);
bool STSPIN32G4_IsReady(void);
bool STSPIN32G4_IsFault(void);
bool STSPIN32G4_I2C_Write(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
void STSPIN32G4_WriteRegister(uint8_t registerValue);
void STSPIN32G4_SetBuckConverterVoltage(uint8_t voltage);
void STSPIN32G4_SetNfaultStatus(void);
bool STSPIN32G4_ClearRegister(void);
void STSPIN32G4_Reset(void);
void STSPIN32G4_ReadStatus(void);

#endif /* STSPIN32G4_H */