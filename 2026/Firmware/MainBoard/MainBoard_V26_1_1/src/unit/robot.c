#include "robot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"

uint16_t adc_val[1];

#define ADC2VOLT 0.008862304688f
#define BATTERY_VOLTAGE_OFFSET 2.0f  // 実測とのズレを補正するオフセット
#define ROCK_SPI_RX_PACKET_SIZE 18U
// この時間(ms)受信できなかったら信号ロストと判定する（ここを変えれば猶予秒数を調整可能）
#define ROCK_SPI_SIGNAL_TIMEOUT_MS 100U

static uint8_t rock_spi_tx_packet[ROCK_SPI_RX_PACKET_SIZE] = {};
static uint8_t rock_spi_rx_packet[ROCK_SPI_RX_PACKET_SIZE];    // 割り込みの受信先
static uint8_t rock_spi_rx_snapshot[ROCK_SPI_RX_PACKET_SIZE];  // メインループ反映用
static volatile uint8_t rock_rx_ready = 0;                     // 新しいフレームを受信したフラグ

// 最後に受信できた時刻(ms)。これからの経過時間でタイムアウトを判定する
static uint32_t rock_last_recv_tick = 0;

static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info);

// 最後の受信から一定時間が過ぎていたら信号ロスト(0)にする
static void Robot_RockUpdateSignalTimeout(RobotInfo* info) {
  if ((HAL_GetTick() - rock_last_recv_tick) > ROCK_SPI_SIGNAL_TIMEOUT_MS) {
    info->status.is_signal_received = 0;
  }
}

// SPI送受信の待ち受けを開始/再開する（常に待ち受け状態を保ち、隙間を作らない）
static void Robot_RockArm(void) {
  HAL_SPI_TransmitReceive_IT(&hspi2, rock_spi_tx_packet, rock_spi_rx_packet,
                             ROCK_SPI_RX_PACKET_SIZE);
}

// 送受信完了割り込み：受信データを退避してフラグを立て、即座に次を待ち受ける
// （重い処理(printf等)はISR内でやらず、反映はメインループに任せる）
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance != SPI2) return;

  memcpy(rock_spi_rx_snapshot, rock_spi_rx_packet, ROCK_SPI_RX_PACKET_SIZE);
  rock_last_recv_tick = HAL_GetTick();
  rock_rx_ready = 1;

  Robot_RockArm();  // 隙間なく次フレームを待ち受け
}

// SPIエラー(オーバーラン等)時も待ち受けを再開して復帰する
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance != SPI2) return;
  Robot_RockArm();
}

static void Robot_RockApplyRecvPacket(RobotInfo* info, const uint8_t* data) {
  info->vel_x.l = data[0];
  info->vel_x.h = data[1];
  info->vel_y.l = data[2];
  info->vel_y.h = data[3];
  info->vel_angular.l = data[4];
  info->vel_angular.h = data[5];
  info->dribble_power = data[6];
  info->kicker.straight = data[7];
  info->kicker.chip = data[8];
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
  UART_HandleTypeDef* md_uarts[4] = {&huart5, &huart6, &huart2, &huart3};
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

  // Rock との SPI を割り込みで常時待ち受け開始（スレーブが常に受信待ちになる）
  Robot_RockBuildTxPacket(self, &self->info);
  Robot_RockArm();

  printf("Robot Initialize Finish\n");
  DigitalOut_Write(&self->led0, 0);
}

void Robot_UpdateSensor(Robot* self) {
  // 電圧センサ値を更新
  self->info.battery_voltage = adc_val[0] * ADC2VOLT + BATTERY_VOLTAGE_OFFSET;  // 0-255 (0-25.5V)
}

// 送信パケットを組み立てる（先頭4byteが実データ、残りは0クリア）
static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info) {
  rock_spi_tx_packet[0] = info->battery_voltage * 10;
  rock_spi_tx_packet[1] = info->dribble_status.data;
  rock_spi_tx_packet[2] = info->kicker_status.cap_val;
  int16_t wheel_scaled[4] = {
      self->omni_drive.vel_wheel_angular[0] * 100,
      self->omni_drive.vel_wheel_angular[1] * 100,
      self->omni_drive.vel_wheel_angular[2] * 100,
      self->omni_drive.vel_wheel_angular[3] * 100};

  for (int i = 0; i < 4; i++) {
    rock_spi_tx_packet[3 + i * 2] = (uint8_t)(wheel_scaled[i] & 0xFF);
    rock_spi_tx_packet[4 + i * 2] = (uint8_t)((wheel_scaled[i] >> 8) & 0xFF);
  }

  for (uint16_t i = 4; i < ROCK_SPI_RX_PACKET_SIZE; i++) {
    rock_spi_tx_packet[i] = 0x00;
  }
}

// メインループから毎周呼ぶ。実際の送受信は割り込みでバックグラウンド実行されているので、
// ここでは「次に送るデータの更新」と「受信済みフレームの反映/タイムアウト判定」だけを行う。
void Robot_RockUpdateSPI(Robot* self, RobotInfo* info) {
  // 次フレームで送る送信データを更新（割り込みが再アーム時にこれを送る）
  Robot_RockBuildTxPacket(self, info);

  if (rock_rx_ready) {
    // 新しいフレームを受信済み → 反映（is_signal_received=1）
    rock_rx_ready = 0;
    Robot_RockApplyRecvPacket(info, rock_spi_rx_snapshot);
  } else {
    // しばらく受信が無ければ信号ロスト判定
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
  if (send_count >= 100) {  // 100msに1回送信 (1秒間に10回)
    UI_Send(&self->ui, self);
    send_count = 0;
  }
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
  OmniDrive_SetVel(&self->omni_drive, info->vel_x.vel, info->vel_y.vel,
                   info->vel_angular.vel);
}

// 2秒周期のSin波でPWM出力（HBピン: PA8 / TIM1_CH1）
void Robot_UpdateHeartBeat(Robot* self) {
  static uint32_t count = 0;
  count = (count + 1) % 2000;
  float duty = (sinf(2.0f * 3.14159265f * count / 2000.0f) + 1.0f) / 2.0f;
  PwmOut_Write(&self->heart_beat, duty);
}
