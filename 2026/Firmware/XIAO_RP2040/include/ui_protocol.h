// MainBoard(STM32, UART4) <-> UI(XIAO RP2040, UART0) 通信プロトコル定義
//
// このファイルは両プロジェクトで同一内容を使う (C/C++ 共通)。変更時は必ず両方を更新すること。
//   UI 側      : XIAO_RP2040/include/ui_protocol.h (正)
//   MainBoard 側: MainBoard_V26_2/src/config/ui_protocol.h (コピー)
//
// 物理層: 250000bps, 8N1
// フレーム: [0xFF][TYPE][LEN][PAYLOAD × LEN][CRC8][0xAA]
//   CRC8 は TYPE, LEN, PAYLOAD に対して計算 (多項式 0x07, 初期値 0x00)
//   多バイト値はすべてリトルエンディアン
#ifndef UI_PROTOCOL_H_
#define UI_PROTOCOL_H_

#include <stdint.h>

#define UI_PROTO_HEADER 0xFFU
#define UI_PROTO_FOOTER 0xAAU
#define UI_PROTO_OVERHEAD 5U  // HEADER + TYPE + LEN + CRC + FOOTER
#define UI_PROTO_MAX_PAYLOAD 32U

// UI がこの時間コマンドを受け取れなかったら MainBoard はテストを中止する [ms]
#define UI_PROTO_LINK_TIMEOUT_MS 200U

// ---------------------------------------------------------------------------
// TYPE 0x01: テレメトリ (MainBoard -> UI, 20ms 周期)
// ---------------------------------------------------------------------------
#define UI_PROTO_TYPE_TELEMETRY 0x01U
#define UI_PROTO_TELEMETRY_LEN 24U

// payload オフセット
#define UI_TLM_BATTERY 0     // uint16 電源電圧 [0.01V]
#define UI_TLM_CAP 2         // uint16 昇圧(コンデンサ)電圧 [V]
#define UI_TLM_FLAGS1 4      // uint8  UI_TLM_F1_*
#define UI_TLM_FLAGS2 5      // uint8  UI_TLM_F2_*
#define UI_TLM_WHEEL 6       // int16 × 4 ホイール実角速度 [0.01rad/s] (motor0..3)
#define UI_TLM_ACCEL_X 14    // int16  加速度X [mg]
#define UI_TLM_ACCEL_Y 16    // int16  加速度Y [mg]
#define UI_TLM_YAW_RATE 18   // int16  ヨー角速度 [0.1deg/s]
#define UI_TLM_YAW 20        // int16  ヨー角 [0.01deg] (-18000..18000)
#define UI_TLM_DRIBBLE 22    // uint8  現在のドリブラー出力 [%]
#define UI_TLM_KICK_ACK 23   // uint8  最後に実行したキックの kick_seq

#define UI_TLM_F1_BALL_DETECTED (1U << 0)  // ボール検知 (フォトセンサ)
#define UI_TLM_F1_BALL_HOLD (1U << 1)      // ボール保持
#define UI_TLM_F1_CHARGE_DONE (1U << 2)    // 充電完了
#define UI_TLM_F1_ROCK_LINK (1U << 3)      // Rock5A から信号受信中 (UI テストは無効)
#define UI_TLM_F1_EMERGENCY (1U << 4)      // 緊急停止
#define UI_TLM_F1_TEST_ACTIVE (1U << 5)    // MainBoard が UI テスト指令を受理中
#define UI_TLM_F1_CHARGING (1U << 6)       // 充電指令中 (0: 放電指令中)
#define UI_TLM_F1_IMU_READY (1U << 7)      // IMU 初期化済み

#define UI_TLM_F2_WHEEL_EMG (1U << 0)    // ホイールユニットが EMG を報告
#define UI_TLM_F2_WHEEL_READY (1U << 1)  // ホイールユニット ready

// ---------------------------------------------------------------------------
// TYPE 0x02: テストコマンド (UI -> MainBoard, 20ms 周期)
// ---------------------------------------------------------------------------
#define UI_PROTO_TYPE_COMMAND 0x02U
#define UI_PROTO_COMMAND_LEN 8U

#define UI_CMD_FLAGS 0         // uint8  UI_CMD_F_*
#define UI_CMD_DRIBBLE 1       // uint8  ドリブラー出力 [%] (0..100)
#define UI_CMD_KICK_POWER 2    // uint8  キック強さ [%] (0..100)
#define UI_CMD_KICK_SEQ 3      // uint8  キック要求ごとに +1 (値が変化したときだけ 1 回キック)
#define UI_CMD_KICK_TYPE 4     // uint8  UI_KICK_*
#define UI_CMD_WHEEL_MASK 5    // uint8  bit0..3 = motor0..3 を回す
#define UI_CMD_WHEEL_TARGET 6  // int16  ホイール目標角速度 [0.01rad/s]

#define UI_CMD_F_TEST_ENABLE (1U << 0)  // 0 のときは他の全フィールドを無視する
#define UI_CMD_F_DRIBBLE_ON (1U << 1)
#define UI_CMD_F_CHARGE (1U << 2)     // 1: 充電, 0: 放電
#define UI_CMD_F_WHEEL_RUN (1U << 3)  // 0 のときホイールは Free

#define UI_KICK_NONE 0U
#define UI_KICK_STRAIGHT 1U
#define UI_KICK_CHIP 2U

// ---------------------------------------------------------------------------
// ヘルパ
// ---------------------------------------------------------------------------
static inline uint8_t UiProto_Crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline void UiProto_PutU16(uint8_t *dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFFU);
  dst[1] = (uint8_t)(v >> 8);
}

static inline uint16_t UiProto_GetU16(const uint8_t *src) {
  return (uint16_t)(src[0] | ((uint16_t)src[1] << 8));
}

// フレームを組み立てる。dst は len + UI_PROTO_OVERHEAD バイト以上。戻り値はフレーム長
static inline uint16_t UiProto_Build(uint8_t *dst, uint8_t type, const uint8_t *payload,
                                     uint8_t len) {
  dst[0] = UI_PROTO_HEADER;
  dst[1] = type;
  dst[2] = len;
  for (uint8_t i = 0; i < len; i++) dst[3 + i] = payload[i];
  dst[3 + len] = UiProto_Crc8(&dst[1], (uint16_t)(len + 2U));
  dst[4 + len] = UI_PROTO_FOOTER;
  return (uint16_t)(len + UI_PROTO_OVERHEAD);
}

// TYPE ごとの規定ペイロード長 (未知の TYPE は 0)
static inline uint8_t UiProto_ExpectedLen(uint8_t type) {
  switch (type) {
    case UI_PROTO_TYPE_TELEMETRY: return UI_PROTO_TELEMETRY_LEN;
    case UI_PROTO_TYPE_COMMAND: return UI_PROTO_COMMAND_LEN;
    default: return 0;
  }
}

// 受信パーサ (1 バイトずつ投入する)。TYPE と LEN の組が規定外なら即座に破棄して再同期する
typedef struct {
  uint8_t state;
  uint8_t type;
  uint8_t len;
  uint8_t idx;
  uint8_t buf[UI_PROTO_MAX_PAYLOAD + 2U];  // TYPE, LEN, PAYLOAD (CRC 計算用に連続配置)
} UiProtoParser;

static inline void UiProto_ParserInit(UiProtoParser *p) { p->state = 0; }

// フレームが完成したら 1 を返し、*type と payload(p->buf + 2), *len が有効になる
static inline uint8_t UiProto_Parse(UiProtoParser *p, uint8_t byte, uint8_t *type,
                                    const uint8_t **payload, uint8_t *len) {
  switch (p->state) {
    case 0:  // HEADER
      if (byte == UI_PROTO_HEADER) p->state = 1;
      break;
    case 1:  // TYPE
      if (byte == UI_PROTO_HEADER) break;  // ヘッダ連続は再同期として扱う
      if (UiProto_ExpectedLen(byte) == 0) {
        p->state = 0;
        break;
      }
      p->type = byte;
      p->buf[0] = byte;
      p->state = 2;
      break;
    case 2:  // LEN
      if (byte != UiProto_ExpectedLen(p->type)) {
        p->state = (byte == UI_PROTO_HEADER) ? 1 : 0;
        break;
      }
      p->len = byte;
      p->buf[1] = byte;
      p->idx = 0;
      p->state = (byte == 0) ? 4 : 3;
      break;
    case 3:  // PAYLOAD
      p->buf[2 + p->idx++] = byte;
      if (p->idx >= p->len) p->state = 4;
      break;
    case 4:  // CRC
      p->state = (byte == UiProto_Crc8(p->buf, (uint16_t)(p->len + 2U))) ? 5 : 0;
      break;
    case 5:  // FOOTER
      p->state = 0;
      if (byte == UI_PROTO_FOOTER) {
        *type = p->type;
        *payload = &p->buf[2];
        *len = p->len;
        return 1;
      }
      break;
    default:
      p->state = 0;
      break;
  }
  return 0;
}

#endif  // UI_PROTOCOL_H_
