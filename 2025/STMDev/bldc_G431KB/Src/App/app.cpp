#include "app.hpp"

#include "RGBLED.hpp"
#include "STSPIN32G4.hpp"

DigitalOut led_alive(LED_ALIVE_GPIO_Port, LED_ALIVE_Pin);
DigitalIn sw_a(SW_A_GPIO_Port, SW_A_Pin);
DigitalIn sw_b(SW_B_GPIO_Port, SW_B_Pin);
DigitalIn sw(SW_GPIO_Port, SW_Pin);
RGBLED rgb;
AS5048A encoder(&hspi1, SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
STSPIN32G4 stspin(&hi2c3);
Flash_EEPROM flash;
PwmOut pwm(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);
BLDCMotor motor(&pwm, &encoder, 8, 0.001);
CAN can(&hfdcan1, 0x555);
Timer canLastRecv;
Timer canSendInterval;

CAN::CANData canSendData;

float targetVelocity = 0;  // radian per sec
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
    uint64_t i64;  // 64bit
    float f32[2];  // 32bit x 2 = 64bit
  } zeroAngle;
  union {
    uint64_t i64;  // 64bit
    float f32[2];  // 32bit x 2 = 64bit [0]=P [1]=I
  } pidGain;
} flashData_t;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
  if (can.getHcan() != &hfdcan1) return;
  CAN::CANData canRecvData;
  can.recv(canRecvData);

  switch (canRecvData.stdId) {
    case 0x00:
      isEmergency = true;
      break;
    case 0x1AA:
      twoByteSplitter vel;

      vel.byte[0] = canRecvData.data[boardId * 2];
      vel.byte[1] = canRecvData.data[boardId * 2 + 1];
      targetVelocity = vel.data;  // radian per sec
      motor.setVelocity(targetVelocity);
      canLastRecv.reset();
      isEmergency = false;
      break;
    case 0x1AB:
      typedef union {
        uint8_t data[4];
        float gain;
      } Gain_t;

      Gain_t p;
      Gain_t i;

      p.data[0] = canRecvData.data[0];
      p.data[1] = canRecvData.data[1];
      p.data[2] = canRecvData.data[2];
      p.data[3] = canRecvData.data[3];

      i.data[0] = canRecvData.data[4];
      i.data[1] = canRecvData.data[5];
      i.data[2] = canRecvData.data[6];
      i.data[3] = canRecvData.data[7];

      pGain = p.gain;
      iGain = i.gain;
      gainChanged = true;
      break;
    default:
      break;
  }
}

void CAN_Data_Output(int16_t vel_rad) {
  CAN::CANData data = {
      .stdId = 0x200 + boardId,
      .data = {
          (uint8_t)(vel_rad & 0xFF),
          (uint8_t)((vel_rad >> 8) & 0xFF)},
  };
  can.send(data);
}

bool checkFlashIsEmpty() {
  flashData_t data;
  bool isEmpty = false;
  printf("checkFlashIsEmpty\n");
  flash.loadFlash(FLASH_START_ADDRESS, (uint64_t*)&data, sizeof(flashData_t));
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
  flash.writeFlash(FLASH_START_ADDRESS, (uint64_t*)&data, sizeof(flashData_t));
  printf("write0: f32[0]:%3.1f f32[1]:%3.1f\n", data.zeroAngle.f32[0], data.zeroAngle.f32[1]);
  printf("write1: f32[0]:%.4f f32[1]:%.4f\n", data.pidGain.f32[0], data.pidGain.f32[1]);
  checkFlashIsEmpty();
}

void setDataFromFlash() {
  flashData_t data;
  flash.loadFlash(FLASH_START_ADDRESS, (uint64_t*)&data, sizeof(flashData_t));
  motor.setAbsoluteZero(data.zeroAngle.f32[0]);
  printf("setZero :%3.1f\n", data.zeroAngle.f32[0]);
  motor.setPIDGain(data.pidGain.f32[0], data.pidGain.f32[1], 0);
  printf("setPIDGain :P:%.4f I:%.4f\n", data.pidGain.f32[0], data.pidGain.f32[1]);
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

void main_app() {
  // A = 0 B = 0 → boardId = 0
  // A = 0 B = 1 → boardId = 1
  // A = 1 B = 0 → boardId = 2
  // A = 1 B = 1 → boardId = 3
  boardId = ((!sw_a.read() << 1) | !sw_b.read());
  printf("boardId:%d\n", boardId);
  for (size_t i = 0; i < 2; i++) {
    rgb.led((rgb_t)(boardId));
    HAL_Delay(50);
    rgb.led(rgb_t::OFF);
    HAL_Delay(50);
  }

  rgb.led(rgb_t::RED);
  // init処理
  stspin.init();
  can.init();
  pwm.init();
  motor.init(false);
  canLastRecv.reset();
  HAL_ADC_Start(&hadc1);

  // FlashからZeroPosを読み込む
  if (checkFlashIsEmpty()) {
    motor.setAbsoluteZero();
    writeFlash(motor.getZeroPos(), pGain, iGain);
  } else {
    setDataFromFlash();
  }

  // モータのPIDゲイン設定
  motor.setPIDGain(0.005, 0.05, 0);

  // イニシャライズ完了
  for (size_t i = 0; i < 3; i++) {
    rgb.led(rgb_t::GREEN);
    HAL_Delay(50);
    rgb.led(rgb_t::OFF);
    HAL_Delay(50);
  }

  while (1) {
    motor.updateEncoder();
    motor.updateAngularVelocity();
    if (canSendInterval.read_ms() > 15) {
      CAN_Data_Output(motor.getAngularVelocity());
      canSendInterval.reset();
    }

    if (isEmergency || canLastRecv.read_ms() > 1000) {
      motor.writePwm(1, 1, 1);
      rgb.led(rgb_t::MAGENTA);
      if (canLastRecv.read_ms() > 2000) canLastRecv.set_ms(2000);
      // printf("Stop %ldms\n", canLastRecv.read_ms());
      targetVelocity = 0;
      isStop = true;
    } else {
      isStop = false;
      if (isStop != isStopPrev) {
        motor.writePwm(0, 0, 0);
        stspin.init();
      }
      if (sw.read() == 0 || gainChanged) {
        // ボタンが押されたらZeroPosの設定をする
        rgb.led(rgb_t::RED);
        motor.writePwm(0, 0, 0);
        wait_ms(1000);
        motor.setAbsoluteZero();
        float zeroPos = motor.getZeroPos();
        writeFlash(zeroPos, pGain, iGain);
        for (size_t i = 0; i < 3; i++) {
          rgb.led(rgb_t::GREEN);
          HAL_Delay(50);
          rgb.led(rgb_t::OFF);
          HAL_Delay(50);
        }
      } else {
        // モータの回転
        motor.drive();

        abs(motor.getTargetVelocity() - motor.getAngularVelocity()) < 3 ? rgb.led(rgb_t::GREEN) : (motor.getTargetVelocity() - motor.getAngularVelocity() > 0 ? rgb.led(rgb_t::MAGENTA) : rgb.led(rgb_t::CYAN));
        // readADC();
        // printf("targetVelocity: %.2f boardId:%d\n", targetVelocity, boardId);
        // printf("vel:%.2f\n", motor.getAngularVelocity());
      }
    }
    isStopPrev = isStop;
  }
}