#ifndef __ROBOT_H_
#define __ROBOT_H_

#include "BufferedSerial.hpp"
#include "Button.hpp"
#include "CAN.hpp"
#include "DigitalInOut.hpp"
#include "dribbler.hpp"
#include "kicker.hpp"
#include "main.h"
#include "omni_drive.hpp"
#include "ui.hpp"

typedef struct {
  // Information Rock5A→STM32 --------------------------
  union {
    struct {
      char l : 8;
      char h : 8;
    };
    int16_t vel;
  } vel_x;
  union {
    struct {
      char l : 8;
      char h : 8;
    };
    int16_t vel;
  } vel_y;
  union {
    struct {
      char l : 8;
      char h : 8;
    };
    int16_t vel;
  } vel_angular;

  uint8_t dribble_power;
  struct {
    uint8_t straight;
    uint8_t chip;
  } kicker;

  union {
    struct {
      char l : 8;
      char h : 8;
    };
    int16_t pos;
  } relative_position_x;

  union {
    struct {
      char l : 8;
      char h : 8;
    };
    int16_t pos;
  } relative_position_y;

  union {
    struct {
      char l : 8;
      char h : 8;
    };
    int16_t theta;
  } relative_theta;

  struct {
    uint8_t x;
    uint8_t y;
  } camera;

  union {
    struct {
      bool emergency_stop : 1;
      bool do_direct_kick : 1;       // ボールセンサが反応した瞬間にストレートキックする
      bool do_direct_chip_kick : 1;  // ボールセンサが反応した瞬間にチップキックする
      bool reserved : 1;
      bool do_charge : 1;           // 0: discharge, 1: do_charge
      bool is_signal_received : 1;  // コントローラから信号が出ているときにtrue
      bool is_ctrl_by_robot : 1;    // (0: RACOON-Ctrl, 1: Robot-local-Ctrl)
                                    // 位置制御をローカルで行うか否か
      bool parity : 1;              // パリティビット 今の所使ってない。
    };
    uint8_t data;
  } status;

  DribbleStatus dribble_status;  // ボール保持とボール検知の統合
  UiStatus ui_status;
  OmniDriveStatus omni_drive_status;
  KickerStatus kicker_status;

  // local
  uint8_t battery_voltage;
  volatile uint16_t photo_sensor_value;
  bool is_under_voltage;
  bool is_robot_stop_ff;  // ロボットのFF速度ベクトルも元にロボットが止まっているかを判断する
  bool is_kicker_charge_mode;
} RobotInfo;

class Robot {
 public:
  Robot();

  RobotInfo info;

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
  BufferedSerial md_serials[4]{
      {&huart2, 128},  // MD1
      {&huart3, 128},  // MD2
      {&huart5, 128},  // MD3
      {&huart6, 128}   // MD4
  };

  OmniDrive omni_drive{md_serials};
  Kicker kicker{&can};
  Dribbler dribbler{&can};
  UI ui{&serial4};

  void Initialize();

  void RockRecvSerial(RobotInfo& info);
  void RockSendSerial(RobotInfo& info);

  // UIからの指示を受け取り、RobotInfoに反映する
  void UpdateFromUi();

  void SendDribble(uint8_t power, bool force_send = false);
  void SendKicker(RobotInfo& info);
  void SendOmniDrive(RobotInfo& info, uint8_t interval);
};

#endif