#include "robot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"

volatile uint16_t adc_val[1];

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
#define ROCK_SPI_ACCEL_SCALE 1000.0f  // [g]     -> int16 (1LSB = 1mg)
// ジャイロFS(±2000dps=約±34.9rad/s、imu.c参照)がint16(最大±32767)に収まるよう900に設定
// (1000だと最大レンジで約±34907となりint16をオーバーフローするため)
#define ROCK_SPI_YAW_RATE_SCALE 900.0f  // [rad/s] -> int16 (1LSB ≈ 0.00111rad/s)
#define ROCK_SPI_YAW_SCALE 10000.0f     // [rad]   -> int16 (1LSB = 0.0001rad)
// SPI がこの時間(ms)完了もエラーもせず BUSY のまま固まったら強制リセットする。
// ソフト NSS のスレーブはビットずれで「完了もエラーもしない BUSY ハング」に
// 陥ることがあり、リセットしないと復帰しない。そのストール検出用。
#define ROCK_SPI_STALL_TIMEOUT_MS 750U
// この時間(ms)有効フレームを受信できなければ Rock5A との通信断とみなし、
// is_signal_received を強制的にクリアする(Rock5Aクラッシュ等の検知用)。
#define ROCK_SPI_SIGNAL_TIMEOUT_MS 300U

// TX/RX共にダブルバッファ (ISR が arm 中のバッファと main が読み書きするバッファを分離)
// TXとRXは常に同じタイミングでフリップするため、arm indexを共用する。
static uint8_t rock_spi_tx_buf[2][ROCK_SPI_FRAME_SIZE];
static uint8_t rock_spi_rx_buf[2][ROCK_SPI_FRAME_SIZE];
static volatile uint8_t rock_spi_tx_arm_idx = 0;

static uint8_t rock_spi_rx_window[ROCK_SPI_RX_WINDOW_SIZE];  // 再同期用 2 フレーム分
static volatile uint8_t rock_rx_ready = 0;
// rock_rx_ready=1 の時点で、完了済みトランザクションのRXデータが入っている
// rock_spi_rx_buf のインデックス(ISRがarm中でなく安全に読める側)。
static volatile uint8_t rock_spi_rx_ready_idx = 0;
static volatile uint8_t rock_rearm_pending = 0;

static uint32_t rock_last_recv_tick = 0;
// 直近に SPI トランザクションが進捗（Arm / 完了）した時刻。
// これが長時間更新されなければ BUSY ハングとみなす。
static volatile uint32_t rock_spi_progress_tick = 0;

// HAL_UART_ErrorCallbackからSerialを引くためのインスタンス
static Robot* robot_instance = NULL;

static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info, uint8_t* dst);
static uint8_t* Robot_RockTxStaging(void);

static inline void Robot_RockPackInt16(uint8_t* dst, int16_t val) {
  dst[0] = (uint8_t)(val & 0xFF);
  dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void Robot_RockArm(void) {
  rock_spi_progress_tick = HAL_GetTick();
  if (HAL_SPI_TransmitReceive_IT(
          &hspi2, rock_spi_tx_buf[rock_spi_tx_arm_idx], rock_spi_rx_buf[rock_spi_tx_arm_idx],
          ROCK_SPI_FRAME_SIZE) != HAL_OK) {
    rock_rearm_pending = 1;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance != SPI2) return;

  // 完了したトランザクションのRXバッファ(このインデックス)は、次のArmでは
  // 使われなくなるのでメインループが安全に読み出せる。
  rock_spi_rx_ready_idx = rock_spi_tx_arm_idx;
  rock_rx_ready = 1;
  rock_spi_tx_arm_idx = 1U - rock_spi_tx_arm_idx;
  Robot_RockArm();
}

// USART割り込み有効時、DMA受信中にFE/NE/ORE等が起きるとHALが受信DMAを停止するため、
// 該当するSerialの受信をリセットして再開する(MDからのフィードバック/UI受信が止まるのを防ぐ)
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  if (robot_instance == NULL) return;
  if (huart == robot_instance->serial4.huart) {
    Serial_Reset(&robot_instance->serial4);
    return;
  }
  for (int i = 0; i < 4; i++) {
    if (huart == robot_instance->md_serials[i].huart) {
      Serial_Reset(&robot_instance->md_serials[i]);
      return;
    }
  }
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
  // UIが手動制御中(is_locked)の間は、dribble_power/kicker/do_chargeをSPI受信値で
  // 上書きしない(Robot_UpdateFromUiで設定したUI側の指示を優先する)
  if (!info->ui_status.is_locked) {
    info->dribble_power = data[6];
    info->kicker.straight = (uint8_t)Constrain(data[7] * 2.55f, 0.0f, 255.0f);
    info->kicker.chip = (uint8_t)Constrain(data[8] * 2.55f, 0.0f, 255.0f);
  }
  info->relative_position_x.l = data[9];
  info->relative_position_x.h = data[10];
  info->relative_position_y.l = data[11];
  info->relative_position_y.h = data[12];
  info->relative_theta.l = data[13];
  info->relative_theta.h = data[14];
  info->camera.x = data[15];
  info->camera.y = data[16];
  if (info->ui_status.is_locked) {
    uint8_t do_charge = info->status.do_charge;
    info->status.data = data[17];
    info->status.do_charge = do_charge;
  } else {
    info->status.data = data[17];
  }
}

static uint8_t* Robot_RockTxStaging(void) {
  return rock_spi_tx_buf[1U - rock_spi_tx_arm_idx];
}

void Robot_Initialize(Robot* self) {
  printf("Robot Initialize Start\n");
  DigitalOut_Init(&self->led0, LED0_GPIO_Port, LED0_Pin);
  DigitalOut_Init(&self->led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&self->led2, LED2_GPIO_Port, LED2_Pin);

  PwmOut_Init(&self->heart_beat, &htim2, TIM_CHANNEL_2);

  DigitalOut_Write(&self->led0, 1);
  HAL_Delay(100);
  DigitalOut_Write(&self->led0, 0);
  HAL_Delay(100);
  DigitalOut_Write(&self->led0, 1);
  HAL_Delay(100);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 1);
  HAL_Delay(10);

  robot_instance = self;
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
  Imu_Calibrate(&self->imu);
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
  dst[1] = info->battery_voltage * 5;
  dst[2] = info->dribble_status.data;
  dst[3] = info->kicker_status.cap_val;
  int16_t wheel_scaled[4] = {
      (int16_t)Constrain(self->omni_drive.vel_wheel_angular[0] * 100.0f, -32767.0f, 32767.0f),
      (int16_t)Constrain(self->omni_drive.vel_wheel_angular[1] * 100.0f, -32767.0f, 32767.0f),
      (int16_t)Constrain(self->omni_drive.vel_wheel_angular[2] * 100.0f, -32767.0f, 32767.0f),
      (int16_t)Constrain(self->omni_drive.vel_wheel_angular[3] * 100.0f, -32767.0f, 32767.0f)};
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

  if (rock_rearm_pending) {
    rock_rearm_pending = 0;
    HAL_SPI_Abort(&hspi2);
    Robot_RockArm();
  } else if ((HAL_GetTick() - rock_spi_progress_tick) > ROCK_SPI_STALL_TIMEOUT_MS) {
    // 完了もエラーもせず BUSY のまま固まった（ソフト NSS スレーブの
    // ビットずれによるハング、または Rock5A 未接続でマスタークロックが
    // 全く来ていない場合も同様に検出される）。強制的に Abort して再 Arm し復帰させる。
    printf("Rock SPI stall detected (no master clock for %ums), re-arming\n",
           ROCK_SPI_STALL_TIMEOUT_MS);
    HAL_SPI_Abort(&hspi2);
    Robot_RockArm();
  }

  primask = __get_PRIMASK();
  __disable_irq();
  uint8_t rx_ready = rock_rx_ready;
  uint8_t rx_idx = rock_spi_rx_ready_idx;
  rock_rx_ready = 0;
  if (primask == 0U) {
    __enable_irq();
  }

  if (rx_ready) {
    Robot_RockRxWindowPush(rock_spi_rx_buf[rx_idx]);
    int16_t payload_offset = Robot_RockFindFrame(rock_spi_rx_window, ROCK_SPI_RX_WINDOW_SIZE);
    if (payload_offset >= 0) {
      rock_last_recv_tick = HAL_GetTick();
      Robot_RockApplyRecvPacket(info, &rock_spi_rx_window[payload_offset]);
    }
  }

  // Rock5Aクラッシュ等で有効フレームが一定時間来なくなった場合、
  // 受信データが古いまま残っていても is_signal_received を強制的にクリアし、
  // main_mode.cの停止判定を確実に発火させる。
  if ((HAL_GetTick() - rock_last_recv_tick) > ROCK_SPI_SIGNAL_TIMEOUT_MS) {
    info->status.is_signal_received = 0;
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
                                  uint8_t do_direct, uint8_t ball_detected_edge,
                                  uint8_t value_changed) {
  if (power == 0) return;
  // do_direct時はボールセンサ反応のエッジで発火、それ以外(即時キック指示)は
  // 前回受信値からの変化がある場合のみ発火する(値が残ったままの自動再発火を防止)
  if (do_direct) {
    if (!ball_detected_edge) return;
  } else if (!value_changed) {
    return;
  }
  Kicker_Kick(kicker, is_straight, power);
}

void Robot_SendKicker(Robot* self, RobotInfo* info) {
  static uint8_t prev_ball_detected = 0;
  static uint8_t prev_chip_power = 0;
  static uint8_t prev_straight_power = 0;

  uint8_t ball_detected = info->dribble_status.is_detected_ball;
  uint8_t ball_detected_edge = ball_detected && !prev_ball_detected;
  uint8_t chip_changed = info->kicker.chip != prev_chip_power;
  uint8_t straight_changed = info->kicker.straight != prev_straight_power;

  // do_direct_straightとdo_direct_chipが同時にセットされるのはプロトコル違反だが、
  // その場合に両方の分岐が抑制され完全に無反応になるのを避けるため、
  // ストレートキックを優先する(チップ側は無効化する)
  uint8_t do_direct_straight = info->status.do_direct_straight;
  uint8_t do_direct_chip = info->status.do_direct_chip;
  if (do_direct_straight && do_direct_chip) {
    do_direct_chip = 0;
  }

  if (!do_direct_straight && info->kicker.chip > 0) {
    Robot_KickIfTriggered(&self->kicker, KICKER_CHIP, info->kicker.chip,
                          do_direct_chip, ball_detected_edge, chip_changed);
  }
  if (!do_direct_chip && info->kicker.straight > 0) {
    Robot_KickIfTriggered(&self->kicker, KICKER_STRAIGHT, info->kicker.straight,
                          do_direct_straight, ball_detected_edge, straight_changed);
  }

  prev_ball_detected = ball_detected;
  prev_chip_power = info->kicker.chip;
  prev_straight_power = info->kicker.straight;
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
