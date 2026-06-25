#ifndef FLASH_EEPROM_HPP
#define FLASH_EEPROM_HPP

#ifdef __cplusplus
#include "main.h"
#include <string.h>

#define FLASH_START_ADDRESS 0x08060000
// ref:https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

// FLASH
// https : // umtkm.github.io/2017/12/19/nucleo-f401-flash-struct/

class Flash_EEPROM {
  public:
    Flash_EEPROM() {}
    ~Flash_EEPROM() {}
    void writeFlash(uint32_t address, uint8_t *data, uint32_t size) {
        HAL_FLASH_Unlock(); /* フラッシュをアンロック */
        _eraseFlash();      /* セクタ7を消去 */
        do {
            /* 1Byteずつフラッシュに書き込む */
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, *data);
        } while (++address, ++data, --size);
        HAL_FLASH_Lock(); /* フラッシュをロック */
    }

    void loadFlash(uint32_t address, uint8_t *data, uint32_t size) {
        memcpy(data, (uint8_t *)address, size);
    }

    void writeFlashTest() {
        flash_eeprom_emulation_data_t data;
        uint32_t address = FLASH_START_ADDRESS;
        loadFlash(address, (uint8_t *)&data, sizeof(flash_eeprom_emulation_data_t));
        printf("Before a:%d, b:%d, c:%d\n", data.a, data.b, data.c);

        data.a += 1;
        data.b += 2;
        data.c += 3;

        writeFlash(address, (uint8_t *)&data, sizeof(flash_eeprom_emulation_data_t)); // フラッシュに書き込み
        loadFlash(address, (uint8_t *)&data, sizeof(flash_eeprom_emulation_data_t));  // フラッシュから読み込み
        printf("After a:%d, b%d, c:%d\n", data.a, data.b, data.c);
    }

    typedef struct {
        uint16_t a;
        uint16_t b;
        uint16_t c;
    } flash_eeprom_emulation_data_t;

  private:
    void _eraseFlash(void) {
        FLASH_EraseInitTypeDef erase;
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;  /* セクタを選ぶ */
        erase.Sector = FLASH_SECTOR_7;              /* セクタ7を指定 */
        erase.NbSectors = 1;                        /* 消すセクタの数．今回は1つだけ */
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 3.3Vで駆動するならこれで */

        uint32_t pageError = 0;

        HAL_FLASHEx_Erase(&erase, &pageError); /* HAL_FLASHExの関数で消去 */
    }
};

#endif
#endif