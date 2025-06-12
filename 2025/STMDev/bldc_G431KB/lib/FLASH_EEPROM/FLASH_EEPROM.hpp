#ifndef FLASH_EEPROM_HPP
#define FLASH_EEPROM_HPP

#ifdef __cplusplus
#include "main.h"
#include <string.h>

#define FLASH_START_ADDRESS 0x0801F800 // Page63: 0x0801 F800 - 0x0801 FFFF (128KB)

class Flash_EEPROM {
  public:
    Flash_EEPROM() {}
    ~Flash_EEPROM() {}

    void writeFlash(uint32_t address, uint64_t *data, uint32_t size) {
        HAL_FLASH_Unlock(); // フラッシュのロックを解除
        FLASH_EraseInitTypeDef erase = {
            .TypeErase = FLASH_TYPEERASE_PAGES, // ページ単位で消去にする
            .Banks = FLASH_BANK_1,              // バンク1を指定
            .Page = 63,                         // ページ63を指定
            .NbPages = 1,                       // 1ページだけ消すので1を指定
        };
        uint32_t pageError = 0;
        HAL_FLASHEx_Erase(&erase, &pageError); // HAL_FLASHExの関数で消去

        // 64bitずつFlashに書き込む
        for (uint32_t i = 0; i < size; i += 8) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, *(data + i / 8));
        }
        HAL_FLASH_Lock(); // フラッシュのロックをかける
    }

    void loadFlash(uint32_t address, uint64_t *data, uint32_t size) {
        memcpy(data, (uint32_t *)address, size);
    }

    void writeFlashTest() {
        FlashData_t data;
        uint32_t address = FLASH_START_ADDRESS;
        loadFlash(address, (uint64_t *)&data, sizeof(FlashData_t)); // フラッシュから読み込み
        printf("load : i16[0]:%3d i16[1]:%3d i16[2]:%3d i16[3]:%3d f32[0]:%3.1f f32[1]:%3.1f\n", data.i16[0], data.i16[1], data.i16[2], data.i16[3], data.f32[0], data.f32[1]);

        if (data.i16[0] == 0xFFFF) {
            printf("Flash is empty.\n");
            data.i16[0] = 0;
            data.i16[1] = 0;
            data.i16[2] = 0;
            data.i16[3] = 0;
            data.f32[0] = 0.0;
            data.f32[1] = 0.0;
        }
        // データ登録
        data.i16[0] += 1;
        data.i16[1] += 2;
        data.i16[2] += 3;
        data.i16[3] += 4;
        data.f32[0] += 0.1f;
        data.f32[1] += 0.2f;

        writeFlash(address, (uint64_t *)&data, sizeof(FlashData_t)); // フラッシュに書き込み
        loadFlash(address, (uint64_t *)&data, sizeof(FlashData_t));  // フラッシュから読み込み
        printf("write: i16[0]:%3d i16[1]:%3d i16[2]:%3d i16[3]:%3d f32[0]:%3.1f f32[1]:%3.1f\n", data.i16[0], data.i16[1], data.i16[2], data.i16[3], data.f32[0], data.f32[1]);
    }

    typedef struct {
        // 64bitごとにしか書き込めないのでデータ分割をする
        union {
            uint64_t i;      // 64bit
            uint16_t i16[4]; // 16bit x 4 = 64bit
        };
        union {
            uint64_t f;   // 64bit
            float f32[2]; // 32bit x 2 = 64bit
        };
    } FlashData_t;
};

#endif
#endif