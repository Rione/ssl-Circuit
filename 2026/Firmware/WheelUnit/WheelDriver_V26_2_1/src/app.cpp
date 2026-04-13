#include "app.hpp"
#include "DigitalInOut/DigitalInOut.hpp"
#include "AS5600/AS5600.hpp"
#include "FLASH_EEPROM/FLASH_EEPROM.hpp"
#include "PWM/PWM.hpp"
#include "Timer/Timer.hpp"
#include "BLDC/BLDC.hpp"
#include <cstdio>

// LEDのグローバル変数
DigitalOut LED_B3(LED_B3_GPIO_Port, LED_B3_Pin);
DigitalOut LED_B2(LED_B2_GPIO_Port, LED_B2_Pin);
DigitalOut LED_B1(LED_B1_GPIO_Port, LED_B1_Pin);
DigitalOut LED_XB(LED_XB_GPIO_Port, LED_XB_Pin);
DigitalOut LED_XG(LED_XG_GPIO_Port, LED_XG_Pin);
DigitalOut LED_XR(LED_XR_GPIO_Port, LED_XR_Pin);

// USWスイッチ
DigitalIn USW(USW_GPIO_Port, USW_Pin);

// AS5600 角度センサー (PA4でのアナログ読取)
AS5600 encoder(&hadc2, ADC_CHANNEL_17);

Flash_EEPROM flash;
PwmOut pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);

Timer canLastRecv;

BLDCMotor motor(&pwm, &encoder, 7, 0.001); // 7極ペア, 制御周期1ms

float targetVelocity = 0; // radian per sec
bool isEmergency = false;
bool isStop, isStopPrev = false;
uint8_t boardId = 0;

bool gainChanged = false;
float pGain = 0.005;
float iGain = 0.05;

typedef union {
    int16_t data;
    uint8_t byte[2];
} twoByteSplitter;

typedef struct {
    union {
        uint64_t i64; // 64bit
        float f32[2]; // 32bit x 2 = 64bit
    } zeroAngle;
    union {
        uint64_t i64; // 64bit
        float f32[2]; // 32bit x 2 = 64bit [0]=P [1]=I
    } pidGain;
} flashData_t;

bool checkFlashIsEmpty() {
    flashData_t data;
    bool isEmpty = false;
    printf("checkFlashIsEmpty\n");
    flash.loadFlash(FLASH_START_ADDRESS, (uint64_t *)&data, sizeof(flashData_t));
    printf("load0 : f32[0]:%3.1f f32[1]:%3.1f\n", data.zeroAngle.f32[0], data.zeroAngle.f32[1]);
    printf("load1 : f32[0]:%.4f f32[1]:%.4f\n", data.pidGain.f32[0], data.pidGain.f32[1]);
    if (data.zeroAngle.i64 == 0xFFFFFFFFFFFFFFFF) {
        printf("Flash is empty.\n");
        isEmpty = true;
    } else {
        printf("Flash is not empty.\n");
    }
    return isEmpty;
}

void writeFlash(float zeroPos, float p, float i) {
    flashData_t data;
    data.zeroAngle.f32[0] = zeroPos;
    data.zeroAngle.f32[1] = 0.0;
    data.pidGain.f32[0] = p;
    data.pidGain.f32[1] = i;
    flash.writeFlash(FLASH_START_ADDRESS, (uint64_t *)&data, sizeof(flashData_t));
    printf("write0: f32[0]:%3.1f f32[1]:%3.1f\n", data.zeroAngle.f32[0], data.zeroAngle.f32[1]);
    printf("write1: f32[0]:%.4f f32[1]:%.4f\n", data.pidGain.f32[0], data.pidGain.f32[1]);
    checkFlashIsEmpty();
}

void readADC() {
    // ADCチャンネルの設定
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    uint16_t adcValue = 0;

    HAL_ADC_PollForConversion(&hadc1, 1000);
    adcValue = HAL_ADC_GetValue(&hadc1);
    printf("ADC:%d\n", adcValue);
}

void Setup(void){
  // PWM初期化
  pwm.init();
  
  // LED初期化
  for (size_t i = 0; i < 2; i++) {
    LED_B3 = 1;
    HAL_Delay(50);
    LED_B3 = 0;
    HAL_Delay(50);
  }

  LED_B1 = 1; // RED情報表示
  
  // ADC開始
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);//
  
  // Flash確認
  if (checkFlashIsEmpty()) {
    writeFlash(0.0, pGain, iGain);
  }

  // 初期化完了表示
  for (size_t i = 0; i < 3; i++) {
    LED_B2 = 1; // GREEN
    HAL_Delay(50);
    LED_B2 = 0;
    HAL_Delay(50);
  }
  
  canLastRecv.reset();
}

void MainLoop(){
  // 通信タイムアウト処理
  if (isEmergency || canLastRecv.read_ms() > 1000) {
    pwm.write(0.5, 0.5, 0.5); // 安全に停止
    LED_B3 = 1;
    LED_B1 = 0;
    HAL_Delay(100);
    LED_B3 = 0;
    LED_B1 = 1;
    HAL_Delay(100);
    if (canLastRecv.read_ms() > 2000) canLastRecv.set_ms(2000);
    printf("Stop %ldms\n", canLastRecv.read_ms());
    targetVelocity = 0;
    isStop = true;
  } else {
    isStop = false;
    
    // ボタンが押されたときの処理
    if (USW.read() == 0 || gainChanged) {
      LED_B1 = 1; // RED
      pwm.write(0, 0, 0);
      wait_ms(1000);
      for (size_t i = 0; i < 3; i++) {
        LED_B2 = 1; // GREEN
        HAL_Delay(50);
        LED_B2 = 0;
        HAL_Delay(50);
      }
      gainChanged = false;
    } else {
      motor.drive();
      LED_B2 = 1; // GREEN
    }
  }
  isStopPrev = isStop;
}
