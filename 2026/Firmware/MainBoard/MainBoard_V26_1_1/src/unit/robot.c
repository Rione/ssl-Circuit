#include "robot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"

uint16_t adc_val[1];

#define ADC2VOLT 0.008862304688f
#define BATTERY_VOLTAGE_OFFSET 2.0f  // 実測とのズレを補正するオフセット

// フレーム構成: [ヘッダ0xFF][ペイロード18byte][フッタ0xAA] = 20byte
// CS(NSS)が無いSPIスレーブはフレーム先頭が分からないので、1フレームより大きい
// 固定長(2フレーム分)を一括受信し、その中からソフトで「FF…(18)…AA」を探して
// 切り出す。一括受信中は取りこぼさず、2フレーム分あれば開始位置がどこにズレても
// 必ず1フレーム丸ごと含まれるため、再アームの隙間でバイトが欠けても復帰できる。
#define ROCK_SPI_HEADER 0xFFU
#define ROCK_SPI_FOOTER 0xAAU
#define ROCK_SPI_PAYLOAD_SIZE 18U
#define ROCK_SPI_FRAME_SIZE (ROCK_SPI_PAYLOAD_SIZE + 2U)   // ヘッダ+ペイロード+フッタ=20
#define ROCK_SPI_RX_BUF_SIZE (ROCK_SPI_FRAME_SIZE * 2U)    // 一括受信バッファ=40(2フレーム)
// この時間(ms)受信できなかったら信号ロストと判定する（ここを変えれば猶予秒数を調整可能）
#define ROCK_SPI_SIGNAL_TIMEOUT_MS 100U

static uint8_t rock_spi_tx_buf[ROCK_SPI_RX_BUF_SIZE] = {};   // 送信(20byteフレーム×2)
static uint8_t rock_spi_rx_buf[ROCK_SPI_RX_BUF_SIZE];        // 割り込みの受信先(40byte)
static uint8_t rock_spi_rx_snapshot[ROCK_SPI_RX_BUF_SIZE];   // メインループ走査用
static volatile uint8_t rock_rx_ready = 0;                   // 新しい一括受信完了フラグ
static volatile uint8_t rock_rearm_pending = 0;             // 再アーム失敗→メインで貼り直す

// 最後に受信できた時刻(ms)。これからの経過時間でタイムアウトを判定する
static uint32_t rock_last_recv_tick = 0;

static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info);

// 最後の受信から一定時間が過ぎていたら信号ロスト(0)にする
static void Robot_RockUpdateSignalTimeout(RobotInfo* info) {
  if ((HAL_GetTick() - rock_last_recv_tick) > ROCK_SPI_SIGNAL_TIMEOUT_MS) {
    info->status.is_signal_received = 0;
  }
}

// 40byte一括の送受信を仕掛ける。失敗したらメインループで貼り直す。
static void Robot_RockArm(void) {
  if (HAL_SPI_TransmitReceive_IT(&hspi2, rock_spi_tx_buf, rock_spi_rx_buf,
                                 ROCK_SPI_RX_BUF_SIZE) != HAL_OK) {
    rock_rearm_pending = 1;  // 失敗→メインで貼り直してフリーズ防止
  }
}

// 送受信完了割り込み：受信データを退避してフラグを立て、即座に次を待ち受ける
// （重い処理(printf/フレーム探索)はISR内でやらず、反映はメインループに任せる）
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance != SPI2) return;

  memcpy(rock_spi_rx_snapshot, rock_spi_rx_buf, ROCK_SPI_RX_BUF_SIZE);
  rock_rx_ready = 1;
  Robot_RockArm();  // 隙間なく次の一括受信へ
}

// SPIエラー(オーバーラン等)時の復帰。HALがフラグ処理済みなので、
// ISR内で直接貼り直さずメインに依頼してフリーズを防ぐ。
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance != SPI2) return;
  rock_rearm_pending = 1;
}

// 一括受信バッファ(40byte)から「FF…(18)…AA」のフレームを探し、見つけたら
// ペイロード先頭オフセットを返す。見つからなければ -1。
static int16_t Robot_RockFindFrame(const uint8_t* buf) {
  for (uint16_t i = 0; i + ROCK_SPI_FRAME_SIZE <= ROCK_SPI_RX_BUF_SIZE; i++) {
    if (buf[i] == ROCK_SPI_HEADER &&
        buf[i + ROCK_SPI_FRAME_SIZE - 1] == ROCK_SPI_FOOTER) {
      return (int16_t)(i + 1);  // ペイロード先頭(ヘッダの次)
    }
  }
  return -1;
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

// 送信パケットを組み立てる
// 20byteフレーム[FF][ペイロード18][AA]を作り、40byteバッファに2連続で詰める
// （受信側と同様、マスターがどの位置から読んでも1フレーム拾えるようにする）
static void Robot_RockBuildTxPacket(Robot* self, RobotInfo* info) {
  uint8_t frame[ROCK_SPI_FRAME_SIZE];
  frame[0] = ROCK_SPI_HEADER;
  frame[1] = info->battery_voltage * 10;
  frame[2] = info->dribble_status.data;
  frame[3] = info->kicker_status.cap_val;
  int16_t wheel_scaled[4] = {
      self->omni_drive.vel_wheel_angular[0] * 100,
      self->omni_drive.vel_wheel_angular[1] * 100,
      self->omni_drive.vel_wheel_angular[2] * 100,
      self->omni_drive.vel_wheel_angular[3] * 100};
  for (int i = 0; i < 4; i++) {
    frame[4 + i * 2] = (uint8_t)(wheel_scaled[i] & 0xFF);
    frame[5 + i * 2] = (uint8_t)((wheel_scaled[i] >> 8) & 0xFF);
  }
  // ペイロード末尾(未使用領域)を0クリア、最後にフッタを書く
  for (uint16_t i = 12; i < ROCK_SPI_FRAME_SIZE - 1; i++) {
    frame[i] = 0x00;
  }
  frame[ROCK_SPI_FRAME_SIZE - 1] = ROCK_SPI_FOOTER;

  // 40byteバッファに同じフレームを2連続でコピー
  memcpy(&rock_spi_tx_buf[0], frame, ROCK_SPI_FRAME_SIZE);
  memcpy(&rock_spi_tx_buf[ROCK_SPI_FRAME_SIZE], frame, ROCK_SPI_FRAME_SIZE);
}

// メインループから毎周呼ぶ。実際の送受信は割り込みでバックグラウンド実行されているので、
// ここでは「次に送るデータの更新」と「受信済みフレームの反映/タイムアウト判定」だけを行う。
void Robot_RockUpdateSPI(Robot* self, RobotInfo* info) {
  // 次フレームで送る送信データを更新（割り込みが再アーム時にこれを送る）
  Robot_RockBuildTxPacket(self, info);

  // 再アームに失敗してSPIが止まっていたら、ここで確実に貼り直す（フリーズ復帰）
  if (rock_rearm_pending) {
    rock_rearm_pending = 0;
    HAL_SPI_Abort(&hspi2);  // 状態をREADYへ戻す
    Robot_RockArm();
  }

  if (rock_rx_ready) {
    rock_rx_ready = 0;
    // 40byteバッファから「FF…(18)…AA」のフレームを探して切り出す
    int16_t payload_offset = Robot_RockFindFrame(rock_spi_rx_snapshot);
    if (payload_offset >= 0) {
      rock_last_recv_tick = HAL_GetTick();  // 有効フレーム受信時刻を更新
      Robot_RockApplyRecvPacket(info, &rock_spi_rx_snapshot[payload_offset]);
    } else {
      // 有効フレーム無し（取りこぼし等）→ 反映せずタイムアウト判定に任せる
      Robot_RockUpdateSignalTimeout(info);
    }
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

// do_direct中はボール検知の立ち上がりエッジ(未検知→検知)でだけ発火させ、
// それ以外はpowerが指定されている限り毎回発火させる。
static void Robot_KickIfTriggered(Kicker* kicker, uint8_t is_straight, uint8_t power,
                                  uint8_t do_direct, uint8_t ball_detected_edge) {
  if (power == 0) return;
  if (do_direct && !ball_detected_edge) return;
  Kicker_Kick(kicker, is_straight, power);
}

// ボールセンサ反応時の自動キック(do_direct)はメイン基板内で発火判定まで行う。
// Rock5Aはdo_direct中ずっと1を送り続けるので、do_direct_straightの値そのものをゲートに使い、
// ボール検知の立ち上がりエッジ(未検知→検知)でだけ発火させる(ボールが離れれば次の検知で再発火できる)。
void Robot_SendKicker(Robot* self, RobotInfo* info) {
  static uint8_t prev_ball_detected = 0;

  uint8_t ball_detected = info->dribble_status.is_detected_ball;
  uint8_t ball_detected_edge = ball_detected && !prev_ball_detected;

  if (!info->status.do_direct_straight) {
    Robot_KickIfTriggered(&self->kicker, KICKER_CHIP, info->kicker.chip,
                          info->status.do_direct_chip, ball_detected_edge);
  }
  if (!info->status.do_direct_chip) {
    Robot_KickIfTriggered(&self->kicker, KICKER_STRAIGHT, info->kicker.straight,
                          info->status.do_direct_straight, ball_detected_edge);
  }

  prev_ball_detected = ball_detected;
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
