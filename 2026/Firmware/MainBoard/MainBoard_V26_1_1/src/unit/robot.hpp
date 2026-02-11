#ifndef __ROBOT_H_
#define __ROBOT_H_

#include "BufferedSerial.hpp"
#include "Button.hpp"
#include "CAN.hpp"
#include "DigitalInOut.hpp"
#include "Timer.hpp"
#include "main.h"
#include "motor_drive.hpp"

typedef struct {
      // Infomation Rock5A→STM32 --------------------------
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

      // Infomation STM32→Rock5A -------------------------
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

} RobotInfo_t;

class Robot {
     public:
      Robot();

      RobotInfo_t info;

      DigitalOut led0 = DigitalOut(LED0_GPIO_Port, LED0_Pin);
      DigitalOut led1 = DigitalOut(LED1_GPIO_Port, LED1_Pin);
      DigitalOut led2 = DigitalOut(LED2_GPIO_Port, LED2_Pin);
      DigitalOut led3 = DigitalOut(LED3_GPIO_Port, LED3_Pin);
      DigitalOut led4 = DigitalOut(LED4_GPIO_Port, LED4_Pin);

      Button sw_imu = Button(IMU_RESET_GPIO_Port, IMU_RESET_Pin);
      Button sw_discharge = Button(DISCHARGE_GPIO_Port, DISCHARGE_Pin);

      CANBus can = CANBus(&hcan1, 0);

      BufferedSerial serial1{&huart1, 128};  // pc
      BufferedSerial serial4{&huart4, 128};  // ui
      BufferedSerial mdSerials[4]{
          {&huart2, 128},  // MD1
          {&huart3, 128},  // MD2
          {&huart5, 128},  // MD3
          {&huart6, 128}   // MD4
      };

      MotorDrive motorDrive{mdSerials};

      void Initialize();

      void RockRecvSerial(RobotInfo_t& info);
      void RockSendSerial(RobotInfo_t& info);
      void UiRecvSerial(RobotInfo_t& info);
      void UiSendSerial(RobotInfo_t& info);
};

#endif