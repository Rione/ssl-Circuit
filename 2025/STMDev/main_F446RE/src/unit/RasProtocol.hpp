#ifndef __RasProtocol__
#define __RasProtocol__

// Pi (ssl-RAVEN-Wing) との UART5 のパケット。仕様は docs/UART_PROTOCOL.md。
//
//   パケット = COBS(ペイロード ‖ CRC16) ‖ 0x00
//
//   テレメトリー (STM32 → Pi): type 0x01、ペイロード 22byte、パケット 26byte
//   指令 (Pi → STM32):         type 0x02、ペイロード 20byte、パケット 24byte
//
// ペイロードの先頭は type、次は seq。CRC は CRC-16/CCITT-FALSE をペイロード全体に
// かけ、リトルエンディアンで後ろに付ける。COBS で符号化すると 0x00 が無くなるので、
// 0x00 がパケットの区切りになる。
//
// Wing の internal/mainboard (crc16.go・cobs.go・uart_frame.go) と同じ計算。
// HAL に依存しない (test/ras_protocol_test.cpp を PC の g++ で動かすため)。

#include <stddef.h>
#include <stdint.h>

namespace ras {

constexpr uint8_t kTypeTelemetry = 0x01;
constexpr uint8_t kTypeCommand = 0x02;

constexpr size_t kTelemetryPayloadSize = 22;
constexpr size_t kCommandPayloadSize = 20;
constexpr size_t kCrcSize = 2;
// COBS で符号化して区切りを付けたパケットの長さ。254byte より短いので、COBS で増えるのは 1byte だけ。
constexpr size_t kTelemetryPacketSize = kTelemetryPayloadSize + kCrcSize + 2;  // 26
constexpr size_t kCommandPacketSize = kCommandPayloadSize + kCrcSize + 2;      // 24
// このモジュールで扱うペイロードの上限 (固定長のバッファの大きさ)。
constexpr size_t kMaxPayload = 32;

constexpr uint8_t kDelimiter = 0x00;

// テレメトリーの status のビット。bit0〜2 は dribbleStatus と同じ。
constexpr uint8_t kStatusSensorMask = 0x07;
constexpr uint8_t kStatusImuValid = 0x08;

// 指令のペイロードの 2〜19 バイト目 (18byte) は、旧形式の FF のあとの 18byte と同じ並び。
constexpr size_t kCommandBody = 2;
constexpr size_t kCommandBodySize = 18;

// CRC-16/CCITT-FALSE (多項式 0x1021、初期値 0xFFFF、反転なし、最後の XOR なし)。
// "123456789" で 0x29B1。
uint16_t crc16(const uint8_t* data, size_t len);

// COBS の符号化。dst は len + len / 254 + 1 以上であること。区切りの 0x00 は付けない。
size_t cobsEncode(const uint8_t* src, size_t len, uint8_t* dst);
// COBS の復号。src は区切りの 0x00 を含まない。形が壊れているか cap に入らなければ false。
bool cobsDecode(const uint8_t* src, size_t len, uint8_t* dst, size_t cap, size_t* outLen);

// payload に CRC を付けて COBS で符号化し、区切りの 0x00 を付けて out に書く。書いた長さを返す。
// len は kMaxPayload 以下、out は len + kCrcSize + 2 以上であること。
size_t buildPacket(const uint8_t* payload, size_t len, uint8_t* out);

enum class OpenResult : uint8_t {
  kOk,
  kBadForm,  // COBS が壊れている・長さが違う・type が違う
  kBadCrc,   // CRC が合わない
};

// 区切りの 0x00 を除いた packet を復号して CRC を確かめ、ペイロードの長さが wantLen、
// type が wantType のときだけ payload (wantLen 以上) に書いて kOk を返す。
OpenResult openPacket(const uint8_t* packet, size_t len, uint8_t wantType, size_t wantLen, uint8_t* payload);

// テレメトリーの中身。値はスケール済みの整数 (送る直前の形)。
struct Telemetry {
  uint8_t seq;
  uint8_t batteryDeciVolts;  // 0.1V 単位
  uint8_t status;            // bit0〜2: dribbleStatus、bit3: imu_valid
  uint8_t capValEstimate;    // 0〜100
  int16_t wheel[4];          // rad/s×100。M0 (55°)、M1 (135°)、M2 (-135°)、M3 (-55°) の順
  int16_t accelX;            // g×1000 (BNO055 の軸のまま)
  int16_t accelY;
  int16_t yawRate;           // rad/s×900。反時計回りが正
  int16_t yaw;               // rad×10000。反時計回りが正、起動時が 0
  uint8_t rxBad;             // 捨てた指令の数 (255 の次は 0)
};

// t を kTelemetryPayloadSize byte のペイロードに詰める (type も書く)。
void packTelemetry(const Telemetry& t, uint8_t* payload);

// 実数 v を v×scale の int16 にする (四捨五入)。範囲の外は上限・下限、NaN は 0。
// オーバーフローで符号が反転すると、Pi 側で逆向きの値に読まれるため。
int16_t scaleToInt16(float v, float scale);

}  // namespace ras

#endif
