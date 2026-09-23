#include "robot.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"

uint16_t adc_val[1];

#define ADC2VOLT 0.008862304688f
#define BATTERY_VOLTAGE_OFFSET 2.0f  // 実測とのズレを補正するオフセット

// フレーム構成: [ヘッダ0xFF][ペイロード19byte][フッタ0xAA] = 21byte
// Rock5A マスターは 1 トランザクション 21byte。スレーブも 21byte で同期させる。
// 再同期用に直近 2 フレーム分(42byte)のスライディングウィンドウを保持する。
#define ROCK_SPI_HEADER 0xFFU
#define ROCK_SPI_FOOTER 0xAAU
#define ROCK_SPI_PAYLOAD_SIZE 19U
#define ROCK_SPI_FRAME_SIZE (ROCK_SPI_PAYLOAD_SIZE + 2U)  // ヘッダ+ペイロード+フッタ=21
#define ROCK_SPI_RX_WINDOW_SIZE (ROCK_SPI_FRAME_SIZE * 2U)

// STM32 → Rock5A 送信ペイロード中のIMUデータのスケール
// (float実数値をint16に変換する際の倍率。Rock5A側では逆数を掛けて復元する)
#define ROCK_SPI_ACCEL_SCALE 1000.0f      // [g]     -> int16 (1LSB = 1mg)
// ジャイロFS(±2000dps=約±34.9rad/s、imu.c参照)がint16(最大±32767)に収まるよう900に設定
// (1000だと最大レンジで約±34907となりint16をオーバーフローするため)
#define ROCK_SPI_YAW_RATE_SCALE 900.0f    // [rad/s] -> int16 (1LSB ≈ 0.00111rad/s)
#define ROCK_SPI_YAW_SCALE 10000.0f       // [rad]   -> int16 (1LSB = 0.0001rad)
// SPI がこの時間(ms)完了もエラーもせず BUSY のまま固まったら強制リセットする。
// ソフト NSS のスレーブはビットずれで「完了もエラーもしない BUSY ハング」に
// 陥ることがあり、リセットしないと復帰しない。そのストール検出用。
#define ROCK_SPI_STALL_TIMEOUT_MS 750U

// TX: ダブルバッファ (ISR が arm 中のバッファと main が更新する staging を分離)
static uint8_t rock_spi_tx_buf[2][ROCK_SPI_FRAME_SIZE];
static volatile uint8_t rock_spi_tx_arm_idx = 0;

static uint8_t rock_spi_rx_xfer[ROCK_SPI_FRAME_SIZE];        // 直近 1 トランザクション分
static uint8_t rock_spi_rx_window[ROCK_SPI_RX_WINDOW_SIZE];  // 再同期用 2 フレーム分
static volatile uint8_t rock_rx_ready = 0;
static volatile uint8_t rock_rearm_pending = 0;

static uint32_t rock_last_recv_tick = 0;
// 直近に SPI トランザクションが進捗（Arm / 完了）した時刻。
// これが長時間更新されなければ BUSY ハングとみなす。
static volatile uint32_t rock_spi_progress_tick = 0;

static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info, uint8_t* dst);
static uint8_t* Robot_RockTxStaging(void);

static inline void Robot_RockPackInt16(uint8_t* dst, int16_t val) {
  dst[0] = (uint8_t)(val & 0xFF);
  dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void Robot_RockArm(void) {
  rock_spi_progress_tick = HAL_GetTick();
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
  info->kicker.straight = data[7] * 2.55;
  info->kicker.chip = data[8] * 2.55;
  info->relative_position_x.l = data[9];
  info->relative_position_x.h = data[10];
  info->relative_position_y.l = data[11];
  info->relative_position_y.h = data[12];
  info->relative_theta.l = data[13];
  info->relative_theta.h = data[14];
  info->camera.x = data[15];
  info->camera.y = data[16];
  info->status.data = data[17];
}

static uint8_t* Robot_RockTxStaging(void) {
  return rock_spi_tx_buf[1U - rock_spi_tx_arm_idx];
}

// SPI2 を待ち時間なしで初期状態に戻す。
// HAL_SPI_Abort は IT 転送中だと TXE/RXNE 割り込みで abort 用 ISR が走るのを
// ビジーループで待つが、スレーブでマスタークロックが来ていないとその割り込みは
// 永久に来ず、TX/RX それぞれ 100ms 相当のタイムアウトまで待ち切る (実測で約225ms
// 制御ループが停止していた)。RCC でペリフェラルごとリセットすれば送信バッファの
// 残りやビットずれも含めて確実に初期化でき、待ちも発生しない。
static void Robot_RockResetSpi(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();
  HAL_NVIC_ClearPendingIRQ(SPI2_IRQn);
  // State が RESET 以外なら HAL_SPI_Init は MspInit(GPIO/NVIC 再設定) を呼ばず、
  // hspi2.Init の内容でレジスタだけを設定し直す
  hspi2.State = HAL_SPI_STATE_READY;
  HAL_SPI_Init(&hspi2);
  if (primask == 0U) {
    __enable_irq();
  }
}

void Robot_Initialize(Robot* self) {
  printf("Robot Initialize Start\n");
  DigitalOut_Init(&self->led0, LED0_GPIO_Port, LED0_Pin);
  DigitalOut_Init(&self->led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&self->led2, LED2_GPIO_Port, LED2_Pin);

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
  Imu_Init(&self->imu);
#if IMU_CALIBRATE_ON_BOOT
  if (Imu_Calibrate(&self->imu)) {
    Imu_SaveCalibration(&self->imu);
  } else if (Imu_LoadCalibration(&self->imu)) {
    printf("IMU: robot was moving during calibration, using stored calibration\n");
  }
#else
  if (!Imu_LoadCalibration(&self->imu)) {
    printf("IMU: no stored calibration, calibrating now\n");
    Imu_Calibrate(&self->imu);
  }
#endif

  rock_spi_tx_arm_idx = 0;
  Robot_RockBuildTxPacket(self, &self->info, rock_spi_tx_buf[0]);
  Robot_RockArm();

  printf("Robot Initialize Finish\n");
  DigitalOut_Write(&self->led0, 0);
}

void Robot_UpdateSensor(Robot* self) {
  self->info.battery_voltage = adc_val[0] * ADC2VOLT + BATTERY_VOLTAGE_OFFSET;
  Imu_Update(&self->imu);
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
    Robot_RockPackInt16(&dst[4 + i * 2], wheel_scaled[i]);
  }

  // IMU: 加速度(xy)[g]・角速度(yaw)[rad/s]・Madgwickフィルタによる姿勢(yaw)[rad]
  Robot_RockPackInt16(&dst[12], (int16_t)(self->imu.accel_x * ROCK_SPI_ACCEL_SCALE));
  Robot_RockPackInt16(&dst[14], (int16_t)(self->imu.accel_y * ROCK_SPI_ACCEL_SCALE));
  Robot_RockPackInt16(&dst[16], (int16_t)(self->imu.yaw_rate * ROCK_SPI_YAW_RATE_SCALE));
  Robot_RockPackInt16(&dst[18], (int16_t)(self->imu.yaw_rad * ROCK_SPI_YAW_SCALE));

  dst[ROCK_SPI_FRAME_SIZE - 1] = ROCK_SPI_FOOTER;
}

void Robot_RockUpdateSPI(Robot* self, RobotInfo* info) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  Robot_RockBuildTxPacket(self, info, Robot_RockTxStaging());
  if (primask == 0U) {
    __enable_irq();
  }

  // Rock5A未接続時はストールが ROCK_SPI_STALL_TIMEOUT_MS 周期で繰り返し検出される。
  // ログが埋もれないよう、同一の切断エピソード中は最初の1回だけ出す(受信成功でリセット)
  static bool stall_logged = false;

  if (rock_rearm_pending) {
    rock_rearm_pending = 0;
    Robot_RockResetSpi();
    Robot_RockArm();
  } else if ((HAL_GetTick() - rock_spi_progress_tick) > ROCK_SPI_STALL_TIMEOUT_MS) {
    // 完了もエラーもせず BUSY のまま固まった（ソフト NSS スレーブの
    // ビットずれによるハング、または Rock5A 未接続でマスタークロックが
    // 全く来ていない場合も同様に検出される）。強制的に Abort して再 Arm し復帰させる。
    if (!stall_logged) {
      stall_logged = true;
      printf("Rock SPI stall detected (no master clock for %ums), re-arming\n",
             ROCK_SPI_STALL_TIMEOUT_MS);
    }
    Robot_RockResetSpi();
    Robot_RockArm();
  }

  if (rock_rx_ready) {
    rock_rx_ready = 0;
    Robot_RockRxWindowPush(rock_spi_rx_xfer);
    int16_t payload_offset = Robot_RockFindFrame(rock_spi_rx_window, ROCK_SPI_RX_WINDOW_SIZE);
    if (payload_offset >= 0) {
      rock_last_recv_tick = HAL_GetTick();
      stall_logged = false;  // 受信成功したので次の切断時はまた1回だけログを出す
      Robot_RockApplyRecvPacket(info, &rock_spi_rx_window[payload_offset]);
    }
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
  (void)interval;
  OmniDrive_SetVelEx(&self->omni_drive, info->vel_x.vel, info->vel_y.vel,
                     info->vel_angular.vel, &self->imu);
}

void Robot_UpdateHeartBeat(Robot* self) {
  static uint32_t count = 0;
  count = (count + 1) % 2000;
  float duty = (sinf(2.0f * 3.14159265f * count / 2000.0f) + 1.0f) / 2.0f;
  PwmOut_Write(&self->heart_beat, duty);
}
