#ifndef __Robot__
#define __Robot__

#include "Average.h"
#include "BufferedSerial.hpp"
#include "Button.hpp"
#include "CAN.hpp"
#include "DMAStream.h"
#include "DigitalInOut.hpp"
#include "FLASH_EEPROM.hpp"
#include "KickerBoard.hpp"
#include "MPU6500.hpp"
#include "BNO055.hpp"
#include "MadgwickAHRS.h"
#include "Median.h"
#include "MotorDriver.hpp"
#include "PWMSingle.hpp"
#include "Timer.hpp"
#include "adc.h"
#include "config.h"
#include "main.h"

#define STRAIGHT 0
#define CHIP 1
#define CHARGE 0
#define DISCHARGE 1

enum class playType {
      NONE,
      NOTIFY,
      START,
      STOP,
      ERROR,
      SUCCESS,
      WARNING,
      INFO,
};
typedef struct {
      // 0. ヘッダ0xFFを受信
      // 1. velX[LSB]
      // 2. velX[MSB]
      // 3. velY[LSB]
      // 4. velY[MSB]
      // 5. velAngler[LSB]
      // 6. velAngler[MSB]
      // 7. dribblePower[0~100] [uint8_t]
      // 8. kickerPowerStraight[0~100] [uint8_t]
      // 9. kickerPowerChip[0~100] [uint8_t]
      // 10. relativePositionX[LSB]
      // 11. relativePositionX[MSB]
      // 12. relativePositionY[LSB]
      // 13. relativePositionY[MSB]
      // 14. relativeTheta[LSB]
      // 15. relativeTheta[MSB]
      // 16. cameraX [uint8_t]
      // 17. cameraY [uint8_t]
      // 18.status
      //   . bit0: emergencyStop
      //   . bit1: doDirectKick
      //   . bit2: doDirectChipKick
      //   . bit3: reserved
      //   . bit4: doCharge
      //   . bit5: isSignalReceived
      //   . bit6: isCtrlByRobot (0: RACOON-Ctrl, 1: Robot-local-Ctrl)
      //   . bit7: parity

      // Infomation RaspberryPi→STM32 --------------------------
      union {
            struct {
                  char L : 8;
                  char H : 8;
            };
            int16_t vel;
      } velX;
      union {
            struct {
                  char L : 8;
                  char H : 8;
            };
            int16_t vel;
      } velY;
      union {
            struct {
                  char L : 8;
                  char H : 8;
            };
            int16_t vel;
      } velAngler;

      uint8_t dribblePower;
      struct {
            uint8_t straight;
            uint8_t chip;
      } kicker;

      union {
            struct {
                  char L : 8;
                  char H : 8;
            };
            int16_t pos;
      } relativePositionX;

      union {
            struct {
                  char L : 8;
                  char H : 8;
            };
            int16_t pos;
      } relativePositionY;

      union {
            struct {
                  char L : 8;
                  char H : 8;
            };
            int16_t theta;
      } relativeTheta;

      struct {
            uint8_t x;
            uint8_t y;
      } camera;

      union {
            struct {
                  bool emergencyStop : 1;
                  bool doDirectKick : 1;      // ボールセンサが反応した瞬間にストレートキックする
                  bool doDirectChipKick : 1;  // ボールセンサが反応した瞬間にチップキックする
                  bool reserved0 : 1;
                  bool doCharge : 1;          // 0: discharge, 1: doCharge
                  bool isSignalReceived : 1;  // コントローラから信号か出ているときにtrue
                  bool isCtrlByRobot : 1;     // (0: RACOON-Ctrl, 1: Robot-local-Ctrl) 位置制御をローカルで行うか否か
                  bool parity : 1;            // パリティビット 今の所使ってない。
            };
            uint8_t data;
      } status;

      // Infomation STM32→RaspberryPi -------------------------
      // serial5, 14 bytes: 0xFF | battery(1) | dribbleStatus(1) | capVal(1)
      //   | FL wheel ω(2) | BL(2) | BR(2) | FR(2) | 0xAA
      // wheel ω: int16 LE, rad/s × 100 (motor ω / GEAR_RATIO)
      union {
            struct {
                  bool isDetectedBall : 1;  // ボール検知（フォトセンサーの検知）2024版のisHolDBall
                  bool isHoldBall : 1;      // ボール保持（ドリブラーの検知とフォトセンサーの検知の論理積）
                  bool isNewDrib : 1;       // 新機体:1
                  uint8_t reserved : 5;
            };
            uint8_t data;
      } dribbleStatus;  // ボール保持とボール検知の統合

      uint8_t batteryVoltage;
      uint8_t capValEstimate;  // 0~100

      // Information STM32→Xiao(UI) -----------------------------
      union {
            struct {
                  bool chargeState : 1;   // stmから送られてくる充電状態
                  uint8_t chargeVal : 7;  // capValEstimate
            };
            uint8_t data;
      } capaData;
      playType buzzer;

      // Information Xiao(UI)→STM32 ---------------------------
      union {
            struct {
                  uint8_t mode : 5;
                  bool emergencyStop : 1;
                  bool chargeStateChange : 1;  // 1.切替、0.切替なし
                  bool kick : 1;               // キック
            };
            uint8_t data;
      } uiStatus;

      // Information MD→STM32 ---------------------------
      struct {
            int16_t velX;
            int16_t velY;

            int16_t motorAngularVelocity[4];
      } mdStatus;

      // local
      volatile uint16_t photoSensorValue;
      bool isUnderVoltage;
      bool isRobotStopFF;  // ロボットのFF速度ベクトルも元にロボットが止まっているかを判断する
      bool isKickerChargeMode;
      union {  // kickerBoardの自認
            struct {
                  bool straight : 1;
                  bool chip : 1;
            };
            uint8_t status;
      } kickerBoardDoDirectStatus;

      float meanVelXBuf[15];
      float meanVelYBuf[15];
      float meanVelAngBuf[15];

      union {
            struct {
                  MPU6500::acc_t acc;
                  MPU6500::gyro_t gyro;
            };
      } mpu_imuOffsets;
      float frontDeg;

      union {
            struct {
                  acc_t acc;    // BNO055の加速度センサ
                  gyro_t gyro;  // BNO055のジャイロセンサ
                  vel_t vel;    // BNO055の速度
            };
      } imuState;
      union {
            struct {
                  acc_t acc;    // BNO055の加速度センサのオフセット
                  gyro_t gyro;  // BNO055のジャイロセンサのオフセット
            };
      } imuOffsets;
      
      

} RobotInfo_t;

class Robot {
     public:
      Robot();

      // values
      RobotInfo_t info;

      // peripherals
      CANBus can = CANBus(&hcan1, 0);

      DigitalOut led0 = DigitalOut(LED0_GPIO_Port, LED0_Pin);
      DigitalOut led1 = DigitalOut(LED1_GPIO_Port, LED1_Pin);
      DigitalOut led2 = DigitalOut(LED2_GPIO_Port, LED2_Pin);
      PwmSingleOut ledH = PwmSingleOut(&htim1, TIM_CHANNEL_1);
      Button swImu = Button(IMU_SW_GPIO_Port, IMU_SW_Pin);
      Button swDischarge = Button(DISCHARGE_SW_GPIO_Port, DISCHARGE_SW_Pin);

      BufferedSerial serial1 = BufferedSerial(&huart1, 128);  // pc
      BufferedSerial serial4 = BufferedSerial(&huart4, 128);  // xiao
      BufferedSerial serial5 = BufferedSerial(&huart5, 256);  // RasPi

      MotorDriver motorDriver = MotorDriver(&can);
      KickerBoard kickerBoard = KickerBoard(&can);

      Average<float> meanVelX = Average(info.meanVelXBuf, 15);
      Average<float> meanVelY = Average(info.meanVelYBuf, 15);
      Average<float> meanVelAngler = Average(info.meanVelAngBuf, 15);

      MPU6500 mpu = MPU6500(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
      MPU6500::acc_t acc;
      MPU6500::gyro_t gyro;
      MPU6500::xyz_t att;

      BNO055 bno = BNO055(&hi2c1);

      Flash_EEPROM flash;

      Madgwick filter;

      Timer manageByUserCounter;  // ユーザーがスイッチでキッカーの充電or放電をしてから15秒以上経過したらPiの指示に従う

      void hardwareInit();
      void rasRecvSerial(RobotInfo_t &info);
      void rasSendSerial(RobotInfo_t &info, uint16_t interval);
      void getSensors(RobotInfo_t *info);

      void sendDribble(uint8_t power, bool forceSend = false);
      void sendKicker(RobotInfo_t &info);
      void sendMotor(RobotInfo_t &info, uint8_t interval);

      void checkRobotRest(RobotInfo_t &info);

      void uiSendSerial(RobotInfo_t &info, uint16_t interval);
      void uiRecvSerial(RobotInfo_t &info);

      void mpuget(RobotInfo_t &info);
      void mpuSetup(RobotInfo_t &info);

      void bnoGet(RobotInfo_t &info);
      void bnoCalibrate();

      void photoThresholdSet();

      inline __attribute__((always_inline)) void heartBeat() {
            static int i = 0;
            i++;
            ledH.write(MyMath::sinDeg(int(i / (!info.isUnderVoltage ? 1 : 5))) / 2 + 0.5);
      }

      inline __attribute__((always_inline)) void setChageDoneSignal(uint16_t value) {
            if (value == 0xFF) {  // Done
                  // info.capValEstimate = 100;
                  kickerBoard.setCapValEstimate(100);
                  led2 = true;
            }
      }

      inline __attribute__((always_inline)) void setKickerBoardChargeMode(uint16_t value) {
            if (value == 0xFF) {  // Done
                  info.isKickerChargeMode = true;
                  led2 = true;
            } else if (value == 0x00) {
                  info.isKickerChargeMode = false;
                  led2 = false;
            }
      }

     private:
      uint16_t photoSensorValueBuff[15];
      Median<uint16_t> medianPhotoValue = Median(photoSensorValueBuff, 15);

      uint8_t batteryValueBuff[15];
      Median<uint8_t> medianBatteryValue = Median(batteryValueBuff, 15);
};

#endif