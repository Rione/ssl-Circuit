#include "robot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"

uint16_t adc_val[1];

#define ADC2VOLT 0.008862304688f
#define BATTERY_VOLTAGE_OFFSET 2.0f  // 実測とのズレを補正するオフセット

// フレーム構成: [ヘッダ0xFF][ペイロード18byte][フッタ0xAA] = 20byte
// Rock5A マスターは 1 トランザクション 20byte。スレーブも 20byte で同期させる。
// 再同期用に直近 2 フレーム分(40byte)のスライディングウィンドウを保持する。
#define ROCK_SPI_HEADER 0xFFU
#define ROCK_SPI_FOOTER 0xAAU
#define ROCK_SPI_PAYLOAD_SIZE 18U
#define ROCK_SPI_FRAME_SIZE (ROCK_SPI_PAYLOAD_SIZE + 2U)  // ヘッダ+ペイロード+フッタ=20
#define ROCK_SPI_RX_WINDOW_SIZE (ROCK_SPI_FRAME_SIZE * 2U)
// この時間(ms)受信できなかったら信号ロストと判定する（ここを変えれば猶予秒数を調整可能）
#define ROCK_SPI_SIGNAL_TIMEOUT_MS 100U

// TX: ダブルバッファ (ISR が arm 中のバッファと main が更新する staging を分離)
static uint8_t rock_spi_tx_buf[2][ROCK_SPI_FRAME_SIZE];
static volatile uint8_t rock_spi_tx_arm_idx = 0;

static uint8_t rock_spi_rx_xfer[ROCK_SPI_FRAME_SIZE];        // 直近 1 トランザクション分
static uint8_t rock_spi_rx_window[ROCK_SPI_RX_WINDOW_SIZE];  // 再同期用 2 フレーム分
static volatile uint8_t rock_rx_ready = 0;
static volatile uint8_t rock_rearm_pending = 0;

static uint32_t rock_last_recv_tick = 0;

static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info, uint8_t* dst);
static uint8_t* Robot_RockTxStaging(void);

static void Robot_RockUpdateSignalTimeout(RobotInfo* info) {
  if ((HAL_GetTick() - rock_last_recv_tick) > ROCK_SPI_SIGNAL_TIMEOUT_MS) {
    info->status.is_signal_received = 0;
  }
}

static void Robot_RockArm(void) {
  if (HAL_SPI_TransmitReceive_IT(
          &hspi2, rock_spi_tx_buf[rock_spi_tx_arm_idx], rock_spi_rx_xfer,
          ROCK_SPI_FRAME_SIZE) != HAL_OK) {
    rock_rearm_pending = 1;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance != SPI2) return;

  rock_rx_ready = 1;
  rock_spi_tx_arm_idx = 1U - rock_spi_tx_arm_idx;
  Robot_RockArm();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance != SPI2) return;
  memset(rock_spi_rx_window, 0, ROCK_SPI_RX_WINDOW_SIZE);
  rock_rearm_pending = 1;
}

static void Robot_RockRxWindowPush(const uint8_t* chunk) {
  memmove(rock_spi_rx_window,
          rock_spi_rx_window + ROCK_SPI_FRAME_SIZE,
          ROCK_SPI_RX_WINDOW_SIZE - ROCK_SPI_FRAME_SIZE);
  memcpy(rock_spi_rx_window + ROCK_SPI_RX_WINDOW_SIZE - ROCK_SPI_FRAME_SIZE, chunk,
         ROCK_SPI_FRAME_SIZE);
}

// スライディングウィンドウ内の最後の有効フレームを返す (見つからなければ -1)
static int16_t Robot_RockFindFrame(const uint8_t* buf, uint16_t buf_size) {
  int16_t last = -1;
  for (uint16_t i = 0; i + ROCK_SPI_FRAME_SIZE <= buf_size; i++) {
    if (buf[i] == ROCK_SPI_HEADER &&
        buf[i + ROCK_SPI_FRAME_SIZE - 1] == ROCK_SPI_FOOTER) {
      last = (int16_t)(i + 1);
    }
  }
  return last;
}

static void Robot_RockApplyRecvPacket(RobotInfo* info, const uint8_t* data) {
  info->vel_x.l = data[0];
  info->vel_x.h = data[1];
  info->vel_y.l = data[2];
  info->vel_y.h = data[3];
  info->vel_angular.l = data[4];
  info->vel_angular.h = data[5];
  info->dribble_power = data[6];
  info->kicker.straight = data[7] * 0.5;
  info->kicker.chip = data[8] * 0.5;
  info->relative_position_x.l = data[9];
  info->relative_position_x.h = data[10];
  info->relative_position_y.l = data[11];
  info->relative_position_y.h = data[12];
  info->relative_theta.l = data[13];
  info->relative_theta.h = data[14];
  info->camera.x = data[15];
  info->camera.y = data[16];
  info->status.data = data[17];
  info->status.is_signal_received = 1;
}

static uint8_t* Robot_RockTxStaging(void) {
  return rock_spi_tx_buf[1U - rock_spi_tx_arm_idx];
}

void Robot_Initialize(Robot* self) {
  printf("Robot Initialize Start\n");
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

  UART_HandleTypeDef* md_uarts[4] = {&huart5, &huart6, &huart2, &huart3};
  Serial_Init(&self->serial4, &huart4, ROBOT_SERIAL_BUF_SIZE);
  for (int i = 0; i < 4; i++) {
    Serial_Init(&self->md_serials[i], md_uarts[i], ROBOT_SERIAL_BUF_SIZE);
  }

  Can_Init(&self->can, &hcan1, 0);

  OmniDrive_Init(&self->omni_drive, self->md_serials);
  Kicker_Init(&self->kicker, &self->can);
  Dribbler_Init(&self->dribbler, &self->can);
  UI_Init(&self->ui, &self->serial4);

  rock_spi_tx_arm_idx = 0;
  Robot_RockBuildTxPacket(self, &self->info, rock_spi_tx_buf[0]);
  Robot_RockArm();

  printf("Robot Initialize Finish\n");
  DigitalOut_Write(&self->led0, 0);
}

void Robot_UpdateSensor(Robot* self) {
  self->info.battery_voltage = adc_val[0] * ADC2VOLT + BATTERY_VOLTAGE_OFFSET;
}

static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info, uint8_t* dst) {
  dst[0] = ROCK_SPI_HEADER;
  dst[1] = info->battery_voltage * 10;
  dst[2] = info->dribble_status.data;
  dst[3] = info->kicker_status.cap_val;
  int16_t wheel_scaled[4] = {
      self->omni_drive.vel_wheel_angular[0] * 100,
      self->omni_drive.vel_wheel_angular[1] * 100,
      self->omni_drive.vel_wheel_angular[2] * 100,
      self->omni_drive.vel_wheel_angular[3] * 100};
  for (int i = 0; i < 4; i++) {
    dst[4 + i * 2] = (uint8_t)(wheel_scaled[i] & 0xFF);
    dst[5 + i * 2] = (uint8_t)((wheel_scaled[i] >> 8) & 0xFF);
  }
  for (uint16_t i = 12; i < ROCK_SPI_FRAME_SIZE - 1; i++) {
    dst[i] = 0x00;
  }
  dst[ROCK_SPI_FRAME_SIZE - 1] = ROCK_SPI_FOOTER;
}

void Robot_RockUpdateSPI(Robot* self, RobotInfo* info) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  Robot_RockBuildTxPacket(self, info, Robot_RockTxStaging());
  if (primask == 0U) {
    __enable_irq();
  }

  if (rock_rearm_pending) {
    rock_rearm_pending = 0;
    HAL_SPI_Abort(&hspi2);
    Robot_RockArm();
  }

  if (rock_rx_ready) {
    rock_rx_ready = 0;
    Robot_RockRxWindowPush(rock_spi_rx_xfer);
    int16_t payload_offset = Robot_RockFindFrame(rock_spi_rx_window, ROCK_SPI_RX_WINDOW_SIZE);
    if (payload_offset >= 0) {
      rock_last_recv_tick = HAL_GetTick();
      Robot_RockApplyRecvPacket(info, &rock_spi_rx_window[payload_offset]);
    } else {
      Robot_RockUpdateSignalTimeout(info);
    }
  } else {
    Robot_RockUpdateSignalTimeout(info);
  }
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

  static uint16_t send_count = 0;
  send_count++;
  if (send_count >= 100) {
    UI_Send(&self->ui, self);
    send_count = 0;
  }
}

void Robot_SendDribble(Robot* self, uint8_t power, uint8_t force_send) {
  Dribbler_Send(&self->dribbler, power, force_send);
}

static void Robot_KickIfTriggered(Kicker* kicker, uint8_t is_straight, uint8_t power,
                                  uint8_t do_direct, uint8_t ball_detected_edge) {
  if (power == 0) return;
  if (do_direct && !ball_detected_edge) return;
  Kicker_Kick(kicker, is_straight, power);
}

void Robot_SendKicker(Robot* self, RobotInfo* info) {
  static uint8_t prev_ball_detected = 0;

  uint8_t ball_detected = info->dribble_status.is_detected_ball;
  uint8_t ball_detected_edge = ball_detected && !prev_ball_detected;

  if (!info->status.do_direct_straight && info->kicker.chip > 0) {
    Robot_KickIfTriggered(&self->kicker, KICKER_CHIP, info->kicker.chip,
                          info->status.do_direct_chip, ball_detected_edge);
  }
  if (!info->status.do_direct_chip && info->kicker.straight > 0) {
    Robot_KickIfTriggered(&self->kicker, KICKER_STRAIGHT, info->kicker.straight,
                          info->status.do_direct_straight, ball_detected_edge);
  }

  prev_ball_detected = ball_detected;
}

void Robot_SendOmniDrive(Robot* self, RobotInfo* info, uint8_t interval) {
  OmniDrive_SetVel(&self->omni_drive, info->vel_x.vel, info->vel_y.vel,
                   info->vel_angular.vel);
}

void Robot_UpdateHeartBeat(Robot* self) {
  static uint32_t count = 0;
  count = (count + 1) % 2000;
  float duty = (sinf(2.0f * 3.14159265f * count / 2000.0f) + 1.0f) / 2.0f;
  PwmOut_Write(&self->heart_beat, duty);
}
