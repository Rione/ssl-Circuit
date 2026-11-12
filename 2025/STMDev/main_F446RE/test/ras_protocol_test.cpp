// src/unit/RasProtocol の試験。PC の g++ で動かす (ファームには入らない)。
//
//   cd test && make
//
// 試験の例のバイト列は、Wing (ssl-RAVEN-Wing の internal/mainboard/uart_test.go の
// goldenCommandPacket・goldenTelemetryPacket) と同じ。片方だけ変えないこと。

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <vector>

#include "RasProtocol.hpp"

static int failures = 0;

#define CHECK(cond)                                                    \
      do {                                                             \
            if (!(cond)) {                                             \
                  printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
                  failures++;                                          \
            }                                                          \
      } while (0)

static std::vector<uint8_t> bytes(std::initializer_list<int> v) {
      std::vector<uint8_t> out;
      for (int x : v) out.push_back((uint8_t)x);
      return out;
}

static std::vector<uint8_t> seqBytes(int from, int to) {
      std::vector<uint8_t> out;
      for (int i = from; i <= to; i++) out.push_back((uint8_t)i);
      return out;
}

static std::vector<uint8_t> cat(std::initializer_list<std::vector<uint8_t>> parts) {
      std::vector<uint8_t> out;
      for (const auto& p : parts) out.insert(out.end(), p.begin(), p.end());
      return out;
}

static void printHex(const char* label, const uint8_t* b, size_t n) {
      printf("  %s:", label);
      for (size_t i = 0; i < n; i++) printf(" %02X", b[i]);
      printf("\n");
}

static void testCrc() {
      const char* s = "123456789";
      CHECK(ras::crc16((const uint8_t*)s, strlen(s)) == 0x29B1);
      CHECK(ras::crc16(nullptr, 0) == 0xFFFF);
}

// COBS の決まった例 (Wikipedia の表)。区切りの 0x00 は含めない。
static void testCobsVectors() {
      struct Case {
            std::vector<uint8_t> in, want;
      };
      const Case cases[] = {
          {bytes({0x00}), bytes({0x01, 0x01})},
          {bytes({0x00, 0x00}), bytes({0x01, 0x01, 0x01})},
          {bytes({0x00, 0x11, 0x00}), bytes({0x01, 0x02, 0x11, 0x01})},
          {bytes({0x11, 0x22, 0x00, 0x33}), bytes({0x03, 0x11, 0x22, 0x02, 0x33})},
          {bytes({0x11, 0x22, 0x33, 0x44}), bytes({0x05, 0x11, 0x22, 0x33, 0x44})},
          {bytes({0x11, 0x00, 0x00, 0x00}), bytes({0x02, 0x11, 0x01, 0x01, 0x01})},
          {seqBytes(1, 254), cat({bytes({0xFF}), seqBytes(1, 254)})},
          {cat({bytes({0x00}), seqBytes(1, 254)}), cat({bytes({0x01, 0xFF}), seqBytes(1, 254)})},
          {seqBytes(1, 255), cat({bytes({0xFF}), seqBytes(1, 254), bytes({0x02, 0xFF})})},
          {cat({seqBytes(2, 255), bytes({0x00})}), cat({bytes({0xFF}), seqBytes(2, 255), bytes({0x01, 0x01})})},
          {cat({seqBytes(3, 255), bytes({0x00, 0x01})}), cat({bytes({0xFE}), seqBytes(3, 255), bytes({0x02, 0x01})})},
      };
      for (const auto& c : cases) {
            std::vector<uint8_t> enc(c.in.size() + c.in.size() / 254 + 1);
            const size_t n = ras::cobsEncode(c.in.data(), c.in.size(), enc.data());
            CHECK(n == c.want.size() && memcmp(enc.data(), c.want.data(), n) == 0);
            CHECK(memchr(enc.data(), 0, n) == nullptr);
            std::vector<uint8_t> dec(c.in.size() + 1);
            size_t m = 0;
            CHECK(ras::cobsDecode(c.want.data(), c.want.size(), dec.data(), dec.size(), &m));
            CHECK(m == c.in.size() && memcmp(dec.data(), c.in.data(), m) == 0);
      }
}

static void testCobsRejectsBroken() {
      uint8_t out[16];
      size_t n = 0;
      const uint8_t delim[] = {0x00};
      const uint8_t cut[] = {0x05, 0x11, 0x22};
      const uint8_t zero[] = {0x03, 0x11, 0x00};
      CHECK(!ras::cobsDecode(delim, sizeof(delim), out, sizeof(out), &n));
      CHECK(!ras::cobsDecode(cut, sizeof(cut), out, sizeof(out), &n));
      CHECK(!ras::cobsDecode(zero, sizeof(zero), out, sizeof(out), &n));
      const uint8_t big[] = {0x05, 0x11, 0x22, 0x33, 0x44};
      CHECK(!ras::cobsDecode(big, sizeof(big), out, 2, &n));
}

static std::vector<uint8_t> unhex(const char* s) {
      std::vector<uint8_t> out;
      unsigned v;
      int k;
      while (sscanf(s, " %2x%n", &v, &k) == 1) {
            out.push_back((uint8_t)v);
            s += k;
      }
      return out;
}

// 指令: VelX 0.5、VelY -0.25、VelAngular 1.0、ドリブラー 1、キック 50、status bit5、seq 7。
static const char* kGoldenCommand = "0B 02 07 F4 01 06 FF E8 03 01 32 01 01 01 01 01 01 01 01 04 20 EB D3 00";
// テレメトリー: 電池 148、status 0b101 + imu_valid、キッカー 77、ホイール 1234 250 -3000 -100、
// 加速度 123 -456、yaw_rate 1350、yaw -7854、seq 200、rx_bad 3。
static const char* kGoldenTelemetry =
    "09 01 C8 94 0D 4D D2 04 FA 06 48 F4 9C FF 7B 0A 38 FE 46 05 52 E1 03 12 3C 00";

static void testGoldenTelemetry() {
      ras::Telemetry t = {};
      t.seq = 200;
      t.batteryDeciVolts = 148;
      t.status = 0x05 | ras::kStatusImuValid;
      t.capValEstimate = 77;
      t.wheel[0] = 1234;
      t.wheel[1] = 250;
      t.wheel[2] = -3000;
      t.wheel[3] = -100;
      t.accelX = 123;
      t.accelY = -456;
      t.yawRate = 1350;
      t.yaw = -7854;
      t.rxBad = 3;
      uint8_t payload[ras::kTelemetryPayloadSize];
      ras::packTelemetry(t, payload);
      uint8_t packet[ras::kTelemetryPacketSize];
      const size_t n = ras::buildPacket(payload, sizeof(payload), packet);
      const auto want = unhex(kGoldenTelemetry);
      const bool same = n == want.size() && memcmp(packet, want.data(), n) == 0;
      CHECK(same);
      if (!same) {
            printHex("got ", packet, n);
            printHex("want", want.data(), want.size());
      }
      CHECK(n == ras::kTelemetryPacketSize);
}

static void testGoldenCommand() {
      // Pi が送ってくる指令のパケットを開くと、旧形式と同じ並びの 18byte が出てくること。
      const auto pkt = unhex(kGoldenCommand);
      CHECK(pkt.size() == ras::kCommandPacketSize);
      uint8_t payload[ras::kCommandPayloadSize];
      CHECK(ras::openPacket(pkt.data(), pkt.size() - 1, ras::kTypeCommand, ras::kCommandPayloadSize, payload) ==
            ras::OpenResult::kOk);
      const uint8_t* d = payload + ras::kCommandBody;
      CHECK(payload[1] == 7);                                  // seq
      CHECK((int16_t)(d[0] | (d[1] << 8)) == 500);             // vel_x 0.5 m/s
      CHECK((int16_t)(d[2] | (d[3] << 8)) == -250);            // vel_y
      CHECK((int16_t)(d[4] | (d[5] << 8)) == 1000);            // vel_angular
      CHECK(d[6] == 1 && d[7] == 50 && d[8] == 0);             // dribble, kick straight, chip
      CHECK(d[17] == 0x20);                                     // status bit5 (is_signal_received)

      // 組み立てると同じバイト列になること (Pi 側の実装と同じ)。
      uint8_t out[ras::kCommandPacketSize];
      CHECK(ras::buildPacket(payload, sizeof(payload), out) == pkt.size());
      CHECK(memcmp(out, pkt.data(), pkt.size()) == 0);
}

static void testOpenRejects() {
      const auto pkt = unhex(kGoldenCommand);
      uint8_t payload[ras::kCommandPayloadSize];
      // 1bit 壊すと CRC で捨てる。
      auto bad = pkt;
      bad[5] ^= 0x10;
      CHECK(ras::openPacket(bad.data(), bad.size() - 1, ras::kTypeCommand, ras::kCommandPayloadSize, payload) ==
            ras::OpenResult::kBadCrc);
      // 自分のテレメトリー (type 0x01) が回り込んで届いても、指令として使わない。
      const auto tel = unhex(kGoldenTelemetry);
      CHECK(ras::openPacket(tel.data(), tel.size() - 1, ras::kTypeCommand, ras::kCommandPayloadSize, payload) ==
            ras::OpenResult::kBadForm);
      // type だけ違う指令の長さのパケット。
      uint8_t p2[ras::kCommandPayloadSize] = {ras::kTypeTelemetry};
      uint8_t out[ras::kCommandPacketSize];
      const size_t n = ras::buildPacket(p2, sizeof(p2), out);
      CHECK(ras::openPacket(out, n - 1, ras::kTypeCommand, ras::kCommandPayloadSize, payload) ==
            ras::OpenResult::kBadForm);
      // 旧形式の指令 (FF で始まる 19byte、0x00 を含む) は形が合わない。
      const uint8_t legacy[] = {0xFF, 0xF4, 0x01};
      CHECK(ras::openPacket(legacy, sizeof(legacy), ras::kTypeCommand, ras::kCommandPayloadSize, payload) !=
            ras::OpenResult::kOk);
}

static void testScale() {
      CHECK(ras::scaleToInt16(1.5f, 900.0f) == 1350);
      CHECK(ras::scaleToInt16(-0.7854f, 10000.0f) == -7854);
      CHECK(ras::scaleToInt16(100.0f, 1000.0f) == 32767);    // 飽和 (符号が反転しない)
      CHECK(ras::scaleToInt16(-100.0f, 1000.0f) == -32768);
      CHECK(ras::scaleToInt16(NAN, 1000.0f) == 0);
}

int main() {
      testCrc();
      testCobsVectors();
      testCobsRejectsBroken();
      testGoldenTelemetry();
      testGoldenCommand();
      testOpenRejects();
      testScale();
      if (failures) {
            printf("%d failure(s)\n", failures);
            return 1;
      }
      printf("ok\n");
      return 0;
}
