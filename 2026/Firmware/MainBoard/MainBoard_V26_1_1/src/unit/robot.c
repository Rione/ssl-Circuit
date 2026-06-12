#include "robot.h"

#include <math.h>
#include <stdio.h>

uint16_t adc_val[1];

#define ADC2VOLT 0.008862304688f

void Robot_Initialize(Robot* self) {
  printf("Robot Initialize Start\n");
  // GPIO 初期化
  DigitalOut_Init(&self->led0, LED0_GPIO_Port, LED0_Pin);
  DigitalOut_Init(&self->led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&self->led2, LED2_GPIO_Port, LED2_Pin);

  DigitalIn_Init(&self->sw_imu, IMU_RESET_GPIO_Port, IMU_RESET_Pin);
  DigitalIn_Init(&self->sw_discharge, DISCHARGE_GPIO_Port, DISCHARGE_Pin);

  PwmOut_Init(&self->heart_beat, &htim1, TIM_CHANNEL_1);

  DigitalOut_Write(&self->led0, 1);
  HAL_Delay(100);
  DigitalOut_Write(&self->led0, 0);
  HAL_Delay(100);
  DigitalOut_Write(&self->led0, 1);
  HAL_Delay(100);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 1);
  HAL_Delay(10);

  // シリアル初期化（DMA受信開始）
  UART_HandleTypeDef* md_uarts[4] = {&huart2, &huart3, &huart5, &huart6};
  Serial_Init(&self->serial4, &huart4, ROBOT_SERIAL_BUF_SIZE);
  for (int i = 0; i < 4; i++) {
    Serial_Init(&self->md_serials[i], md_uarts[i], ROBOT_SERIAL_BUF_SIZE);
  }

  // CAN初期化
  Can_Init(&self->can, &hcan1, 0);

  // ユニット初期化（シリアルとCAN初期化後に呼ぶ）
  OmniDrive_Init(&self->omni_drive, self->md_serials);
  Kicker_Init(&self->kicker, &self->can);
  Dribbler_Init(&self->dribbler, &self->can);
  UI_Init(&self->ui, &self->serial4);

  printf("Robot Initialize Finish\n");
  DigitalOut_Write(&self->led0, 0);
}

void Robot_UpdateSensor(Robot* self) {
  // 電圧センサ値を更新
  self->info.battery_voltage = adc_val[0] * ADC2VOLT;  // 0-255 (0-25.5V)
  printf("adc_val: %d, battery_voltage: %d\n", adc_val[0], self->info.battery_voltage);
}

void Robot_RockRecvSerial(Robot* self, RobotInfo* info) {
  // if (self->serial1.huart->hdmarx == NULL) return;  // USART1 DMA未設定

  // static uint8_t recv_data[18];
  // static uint8_t index = 0;

  // if (!Serial_Available(&self->serial1)) return;

  // uint8_t recv_byte = Serial_Read(&self->serial1);

  // if (index == 0) {
  //   if (recv_byte == 0xFF) {
  //     index++;
  //   }
  // } else if (index == 19) {
  //   if (recv_byte == 0xAA) {
  //     info->vel_x.l = recv_data[0];
  //     info->vel_x.h = recv_data[1];
  //     info->vel_y.l = recv_data[2];
  //     info->vel_y.h = recv_data[3];
  //     info->vel_angular.l = recv_data[4];
  //     info->vel_angular.h = recv_data[5];
  //     info->dribble_power = recv_data[6];
  //     info->kicker.straight = recv_data[7];
  //     info->kicker.chip = recv_data[8];
  //     info->relative_position_x.l = recv_data[9];
  //     info->relative_position_x.h = recv_data[10];
  //     info->relative_position_y.l = recv_data[11];
  //     info->relative_position_y.h = recv_data[12];
  //     info->relative_theta.l = recv_data[13];
  //     info->relative_theta.h = recv_data[14];
  //     info->camera.x = recv_data[15];
  //     info->camera.y = recv_data[16];
  //     info->status.data = recv_data[17];
  //   }
  //   index = 0;
  // } else {
  //   recv_data[index - 1] = recv_byte;
  //   index++;
  // }
}

void Robot_RockSendSerial(Robot* self, RobotInfo* info) {
  // if (self->serial1.huart->hdmatx == NULL) return;  // USART1 DMA未設定

  // static Timer timer = {0};
  // if (Timer_ReadMs(&timer) < 100) return;

  // // ヘッダ(4バイト) + データ(3バイト) を1回で送信
  // uint8_t send_buf[7];
  // send_buf[0] = 0xFF;
  // send_buf[1] = 0x00;
  // send_buf[2] = 0xFF;
  // send_buf[3] = 0x00;
  // send_buf[4] = info->dribble_status.data;
  // send_buf[5] = info->battery_voltage;
  // send_buf[6] = info->kicker_status.cap_val_estimate;

  // Serial_Write(&self->serial1, send_buf, sizeof(send_buf));
  // Timer_Reset(&timer);
}

void Robot_UpdateFromUi(Robot* self) {
  UI_Recv(&self->ui, &self->info.ui_status);

  if (!self->info.ui_status.is_locked) {
    if (self->info.ui_status.dribble) {
      self->info.dribble_power = self->info.ui_status.dribbler_power * 10;
    }

    if (self->info.ui_status.straight_kick) {
      self->info.kicker.straight = self->info.ui_status.kicker_power;
    } else if (self->info.ui_status.chip_kick) {
      self->info.kicker.chip = self->info.ui_status.kicker_power;
    }

    if (self->info.ui_status.charge) {
      self->info.status.do_charge = 1;
    } else if (self->info.ui_status.discharge) {
      self->info.status.do_charge = 0;
    }
  }

  UI_Send(&self->ui, self);
}

void Robot_SendDribble(Robot* self, uint8_t power, uint8_t force_send) {
  Dribbler_Send(&self->dribbler, power, force_send);
}

void Robot_SendKicker(Robot* self, RobotInfo* info) {
  if (info->kicker.straight > 0) {
    Kicker_Kick(&self->kicker, KICKER_STRAIGHT, info->kicker.straight,
                info->status.do_direct_kick);
  } else if (info->kicker.chip > 0) {
    Kicker_Kick(&self->kicker, KICKER_CHIP, info->kicker.chip,
                info->status.do_direct_chip_kick);
  } else {
    if (info->status.do_direct_kick != info->kicker_status.do_direct_straight &&
        info->kicker_status.do_direct_straight) {
      Kicker_CancelDirect(&self->kicker, KICKER_STRAIGHT);
    }
    if (info->status.do_direct_chip_kick != info->kicker_status.do_direct_chip &&
        info->kicker_status.do_direct_chip) {
      Kicker_CancelDirect(&self->kicker, KICKER_CHIP);
    }
  }
}

void Robot_SendOmniDrive(Robot* self, RobotInfo* info, uint8_t interval) {
  static uint8_t send_count = 0;
  send_count++;
  if (send_count % interval == 0) {
    OmniDrive_SetVel(&self->omni_drive, info->vel_x.vel, info->vel_y.vel,
                     info->vel_angular.vel);
  }
}

// 2秒周期のSin波でPWM出力（HBピン: PA8 / TIM1_CH1）
void Robot_UpdateHeartBeat(Robot* self) {
  static uint32_t count = 0;
  count = (count + 1) % 2000;
  float duty = (sinf(2.0f * 3.14159265f * count / 2000.0f) + 1.0f) / 2.0f;
  PwmOut_Write(&self->heart_beat, duty);
}
