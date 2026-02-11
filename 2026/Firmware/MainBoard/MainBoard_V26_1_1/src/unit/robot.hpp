#ifndef __ROBOT_H_
#define __ROBOT_H_

#include "BufferedSerial.hpp"
#include "Button.hpp"
#include "CAN.hpp"
#include "DigitalInOut.hpp"
#include "Timer.hpp"
#include "kicker.hpp"
#include "main.h"
#include "motor_drive.hpp"
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
                  bool reserved0 : 1;
                  bool do_charge : 1;           // 0: discharge, 1: do_charge
                  bool is_signal_received : 1;  // コントローラから信号が出ているときにtrue
                  bool is_ctrl_by_robot : 1;    // (0: RACOON-Ctrl, 1: Robot-local-Ctrl) 位置制御をローカルで行うか否か
                  bool parity : 1;              // パリティビット 今の所使ってない。
            };
            uint8_t data;
      } status;

      // Information STM32→Rock5A -------------------------
      union {
            struct {
                  bool is_detected_ball : 1;  // ボール検知（フォトセンサーの検知）2024版のis_hold_ball
                  bool is_hold_ball : 1;      // ボール保持（ドリブラーの検知とフォトセンサーの検知の論理積）
                  bool is_new_drib : 1;       // 新機体:1
                  uint8_t reserved : 5;
            };
            uint8_t data;
      } dribble_status;  // ボール保持とボール検知の統合

      uint8_t battery_voltage;
      uint8_t cap_val_estimate;  // 0~100

      // Information STM32→Xiao(UI) -----------------------------
      union {
            struct {
                  bool charge_state : 1;   // stmから送られてくる充電状態
                  uint8_t charge_val : 7;  // cap_val_estimate
            };
            uint8_t data;
      } capa_data;

      // Information Xiao(UI)→STM32 ---------------------------
      union {
            struct {
                  uint8_t mode : 5;
                  bool emergency_stop : 1;
                  bool charge_state_change : 1;  // 1.切替、0.切替なし
                  bool kick : 1;                 // キック
            };
            uint8_t data;
      } ui_status;

      // Information MD→STM32 ---------------------------
      struct {
            int16_t vel_x;
            int16_t vel_y;

            int16_t motor_angular_velocity[4];
      } md_status;

      // local
      volatile uint16_t photo_sensor_value;
      bool is_under_voltage;
      bool is_robot_stop_ff;  // ロボットのFF速度ベクトルも元にロボットが止まっているかを判断する
      bool is_kicker_charge_mode;
      union {  // kicker_boardの自認
            struct {
                  bool straight : 1;
                  bool chip : 1;
            };
            uint8_t status;
      } kicker_board_do_direct_status;

      float mean_vel_x_buf[15];
      float mean_vel_y_buf[15];
      float mean_vel_ang_buf[15];

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

      MotorDrive motorDrive{md_serials};
      Kicker kicker{&can};
      UI ui{&serial4};

      void Initialize();

      void RockRecvSerial(RobotInfo& info);
      void RockSendSerial(RobotInfo& info);
};

#endif