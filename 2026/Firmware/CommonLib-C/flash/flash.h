#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>

/* =========================================================================
 * MCU family detection
 *
 * STM32F303K8 : 64KB flash, 2KB pages (pages 0-31)
 *               Write unit: WORD (4 bytes)
 *               User area  : page 31 @ 0x0800F800–0x0800FFFF
 *
 * STM32F446RE : 512KB flash, sector-based erase
 *               Write unit: WORD (4 bytes)
 *               User area  : sector 7 @ 0x08060000–0x0807FFFF (128KB)
 *               NOTE: The entire 128KB sector is erased on each write.
 *
 * STM32G431   : 128KB flash, 2KB pages (pages 0-63), single-bank
 *               Write unit: DOUBLEWORD (8 bytes), address must be 8-byte aligned
 *               User area  : page 63 @ 0x0801F800–0x0801FFFF
 * ======================================================================= */

#if defined(STM32F303x8) || defined(STM32F302x8) || defined(STM32F3xx)
  #include "stm32f3xx_hal.h"
  #define FLASH_USER_START_ADDR  0x0800F800U
  #define FLASH_USER_END_ADDR    0x0800FFFFU
  #define FLASH_FAMILY_F3

#elif defined(STM32F446xx) || defined(STM32F4xx)
  #include "stm32f4xx_hal.h"
  #define FLASH_USER_START_ADDR  0x08060000U
  #define FLASH_USER_END_ADDR    0x0807FFFFU
  #define FLASH_FAMILY_F4

#elif defined(STM32G431xx) || defined(STM32G4xx)
  #include "stm32g4xx_hal.h"
  #define FLASH_USER_START_ADDR  0x0801F800U
  #define FLASH_USER_END_ADDR    0x0801FFFFU
  #define FLASH_FAMILY_G4

#else
  #error "flash.h: unsupported MCU. Define STM32F303x8, STM32F446xx, or STM32G431xx."
#endif

/* -------------------------------------------------------------------------
 * Flash_WriteData
 *
 * Erases the user page/sector, then writes 'size' bytes from 'data'
 * to flash starting at 'address'.
 * ------------------------------------------------------------------------- */
static inline HAL_StatusTypeDef Flash_WriteData(uint32_t address,
                                                const void *data, size_t size) {
  if (address < FLASH_USER_START_ADDR ||
      (address + size) > (FLASH_USER_END_ADDR + 1U)) {
    return HAL_ERROR;
  }

  HAL_FLASH_Unlock();

  /* --- Erase ------------------------------------------------------------ */
  FLASH_EraseInitTypeDef eraseInit = {0};
  uint32_t pageError = 0;

#if defined(FLASH_FAMILY_F3)
  eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = FLASH_USER_START_ADDR;
  eraseInit.NbPages     = 1;

#elif defined(FLASH_FAMILY_F4)
  eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
  eraseInit.Sector       = FLASH_SECTOR_7;
  eraseInit.NbSectors    = 1;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7–3.6V */

#elif defined(FLASH_FAMILY_G4)
  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.Banks     = FLASH_BANK_1;
  eraseInit.Page      = 63U;
  eraseInit.NbPages   = 1;
#endif

  if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return HAL_ERROR;
  }

  /* --- Write ------------------------------------------------------------ */
  const uint8_t *src = (const uint8_t *)data;
  size_t written = 0;

#if defined(FLASH_FAMILY_F3) || defined(FLASH_FAMILY_F4)
  /* F3 / F4: write in 32-bit (WORD) units */
  while (written < size) {
    uint32_t word = 0xFFFFFFFFU;
    size_t chunk = ((size - written) < 4U) ? (size - written) : 4U;
    for (size_t b = 0; b < chunk; b++) {
      ((uint8_t *)&word)[b] = src[written + b];
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          address + written, (uint64_t)word) != HAL_OK) {
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
    written += 4U;
  }

#elif defined(FLASH_FAMILY_G4)
  /* G4: write in 64-bit (DOUBLEWORD) units; address must be 8-byte aligned.
   * Remaining bytes (< 8) are padded with 0xFF (= erased state). */
  while (written < size) {
    uint64_t dword = 0xFFFFFFFFFFFFFFFFULL;
    size_t chunk = ((size - written) < 8U) ? (size - written) : 8U;
    for (size_t b = 0; b < chunk; b++) {
      ((uint8_t *)&dword)[b] = src[written + b];
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                          address + written, dword) != HAL_OK) {
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
    written += 8U;
  }
#endif

  HAL_FLASH_Lock();
  return HAL_OK;
}

/* -------------------------------------------------------------------------
 * Flash_ReadData
 *
 * Reads 'size' bytes from flash at 'address' into 'buffer'.
 * Flash is memory-mapped; no special read procedure is needed.
 * ------------------------------------------------------------------------- */
static inline void Flash_ReadData(uint32_t address, void *buffer, size_t size) {
  const uint8_t *src = (const uint8_t *)address;
  uint8_t *dst = (uint8_t *)buffer;
  for (size_t i = 0; i < size; i++) {
    dst[i] = src[i];
  }
}

#endif /* FLASH_H_ */
