#include "stspin32g4.h"

static I2C_HandleTypeDef* s_hi2c = NULL;
static DigitalOut wake;
static DigitalIn ready;
static DigitalIn nFault;

void STSPIN32G4_Init(I2C_HandleTypeDef* hi2c) {
  s_hi2c = hi2c;
  DigitalOut_Init(&wake, WAKE_GPIO_Port, WAKE_Pin);
  DigitalIn_Init(&ready, READY_GPIO_Port, READY_Pin);
  DigitalIn_Init(&nFault, nFAULT_GPIO_Port, nFAULT_Pin);
  DigitalOut_Write(&wake, 1);

  /* Match original C++ init() sequence. */
  STSPIN32G4_Reset();
  HAL_Delay(10);
  STSPIN32G4_SetBuckConverterVoltage(8);
  STSPIN32G4_SetNfaultStatus();
  HAL_Delay(10);
  STSPIN32G4_ClearRegister();
}

bool STSPIN32G4_I2C_Write(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
  uint8_t i2c_data[1] = {data};
  if (s_hi2c == NULL) {
    return false;
  }
  if (HAL_I2C_Mem_Write(s_hi2c, deviceAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, i2c_data, 1, HAL_MAX_DELAY) != HAL_OK) {
    return false;
  }
  return true;
}

void STSPIN32G4_WriteRegister(uint8_t registerValue) {
  if (s_hi2c == NULL) return;
  uint8_t data = registerValue;
  HAL_I2C_Master_Transmit(s_hi2c, STSPING4_CONTROLER_ADDR << 1, &data, 1, 100);
}

void STSPIN32G4_SetBuckConverterVoltage(uint8_t voltage) {
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
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b11110000);
  printf("unlock!! READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, POWMNG_REG, value);
  printf("Writereg READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b00000000);
  printf("lock!!   READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
}

void STSPIN32G4_SetNfaultStatus(void) {
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

  bool i2c_status = false;
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b11110000);
  printf("unlock!! READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, NFAULT_REG, value);
  printf("Writereg READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b00000000);
  printf("lock!!   READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
}

bool STSPIN32G4_ClearRegister(void) {
  bool i2c_status = false;
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, CLEAR__REG, 0b11111111);
  return i2c_status;
}

void STSPIN32G4_Reset(void) {
  bool i2c_status = false;
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b11110000);
  printf("unlock!! READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, RESET_REG, 0xFF);
  printf("Writereg READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
  i2c_status = STSPIN32G4_I2C_Write(STSPING4_CONTROLER_ADDR, LOCK_REG, 0b00000000);
  printf("lock!!   READY:%d NFAULT:%d I2C_success:%d\n", DigitalIn_Read(&ready), DigitalIn_Read(&nFault), i2c_status);
  HAL_Delay(10);
}

void STSPIN32G4_ReadStatus(void) {
  if (s_hi2c == NULL) return;
  uint8_t data[1];
  uint8_t addr = STATUS_REG;
  HAL_I2C_Master_Transmit(s_hi2c, STSPING4_CONTROLER_ADDR << 1, &addr, 1, 100);
  HAL_I2C_Master_Receive(s_hi2c, STSPING4_CONTROLER_ADDR << 1, data, 1, 100);
  printf("LOCK:%d RESET:%d VDS_P:%d THSD:%d VCC_UVLO:%d\n",
         (data[0] >> 7) & 0x01,
         (data[0] >> 3) & 0x01,
         (data[0] >> 2) & 0x01,
         (data[0] >> 1) & 0x01,
         (data[0] >> 0) & 0x01);
}