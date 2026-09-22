#ifndef __ROBOT_H_
#define __ROBOT_H_

#include <stdint.h>

#include "adc.h"
#include "can_bus.h"
#include "digitalinout.h"
#include "dribbler.h"
#include "kicker.h"
#include "main.h"
#include "omni_drive.h"
#include "pwm_out.h"
#include "serial.h"
#include "timer.h"
#include "ui.h"

#define ROBOT_SERIAL_BUF_SIZE 128

typedef struct {
  // Rock5A → STM32 受信データ --------------------------
  union {
    struct {
      uint8_t l;
      uint8_t h;
    };
    int16_t vel;
  } vel_x;
  union {
    struct {
      uint8_t l;
      uint8_t h;
    };
    int16_t vel;
  } vel_y;
  union {
    struct {
      uint8_t l;
      uint8_t h;
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
      uint8_t l;
      uint8_t h;
    };
    int16_t pos;
  } relative_position_x;
  union {
    struct {
      uint8_t l;
      uint8_t h;
    };
    int16_t pos;
  } relative_position_y;
  union {
    struct {
      uint8_t l;
      uint8_t h;
    };
    int16_t theta;
  } relative_theta;

  struct {
    uint8_t x;
    uint8_t y;
  } camera;

  union {
    struct {
      uint8_t emergency_stop : 1;
      uint8_t do_direct_straight : 1;   // ボールセンサ反応時にストレートキック
      uint8_t do_direct_chip : 1;  // ボールセンサ反応時にチップキック
      uint8_t reserved : 1;
      uint8_t do_charge : 1;           // 0: discharge, 1: charge
      uint8_t is_signal_received : 1;  // コントローラから信号受信中
      uint8_t is_ctrl_by_robot : 1;    // (0: RACOON-Ctrl, 1: Robot-local-Ctrl)
      uint8_t parity : 1;
    };
    uint8_t data;
  } status;

  DribbleStatus dribble_status;
  UiStatus ui_status;
  OmniDriveStatus omni_drive_status;
  KickerStatus kicker_status;

  // ローカル情報
  uint8_t battery_voltage;
  volatile uint16_t photo_sensor_value;
  uint8_t is_under_voltage;
  uint8_t is_robot_stop_ff;
  uint8_t is_kicker_charge_mode;
} RobotInfo;

typedef struct Robot {
  RobotInfo info;

  DigitalOut led0, led1, led2;
  DigitalIn sw_imu, sw_discharge;
  PwmOut heart_beat;

  CanBus can;
  Serial serial1;        // PC
  Serial serial4;        // UI
  Serial md_serials[4];  // MD1-4

  OmniDrive omni_drive;
  Kicker kicker;
  Dribbler dribbler;
  UI ui;
} Robot;

void Robot_Initialize(Robot* self);

void Robot_RockUpdateSPI(Robot* self, RobotInfo* info);

void Robot_UpdateFromUi(Robot* self);

void Robot_UpdateSensor(Robot* self);

void Robot_SendDribble(Robot* self, uint8_t power, uint8_t force_send);
void Robot_SendKicker(Robot* self, RobotInfo* info);
void Robot_SendOmniDrive(Robot* self, RobotInfo* info, uint8_t interval);
void Robot_UpdateHeartBeat(Robot* self);

#endif  // __ROBOT_H_
