#include "RasProtocol.hpp"

#include <math.h>
#include <string.h>

namespace ras {

uint16_t crc16(const uint8_t* data, size_t len) {
      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (int b = 0; b < 8; b++) {
                  crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
            }
      }
      return crc;
}

size_t cobsEncode(const uint8_t* src, size_t len, uint8_t* dst) {
      size_t codeAt = 0;  // 今のブロックの長さを書く位置
      size_t n = 1;       // 次に書く位置
      uint8_t code = 1;
      for (size_t i = 0; i < len; i++) {
            const uint8_t b = src[i];
            if (b != 0) {
                  dst[n++] = b;
                  code++;
            }
            if (b == 0 || code == 0xFF) {
                  dst[codeAt] = code;
                  code = 1;
                  codeAt = n;
                  // 0x00 を含まない 254byte で src が終わったときは、次のブロックを作らない
                  if (b == 0 || i + 1 < len) {
                        n++;
                  }
            }
      }
      if (codeAt < n) {
            dst[codeAt] = code;
      }
      return n;
}

bool cobsDecode(const uint8_t* src, size_t len, uint8_t* dst, size_t cap, size_t* outLen) {
      size_t n = 0;
      size_t i = 0;
      while (i < len) {
            const size_t code = src[i];
            if (code == 0) {
                  return false;
            }
            i++;
            const size_t end = i + code - 1;
            if (end > len || n + code - 1 > cap) {
                  return false;
            }
            for (; i < end; i++) {
                  if (src[i] == 0) {
                        return false;
                  }
                  dst[n++] = src[i];
            }
            // 254byte の長いブロック (0xFF) と、最後のブロックの後ろには 0x00 が無い
            if (code != 0xFF && i < len) {
                  if (n >= cap) {
                        return false;
                  }
                  dst[n++] = 0;
            }
      }
      *outLen = n;
      return true;
}

size_t buildPacket(const uint8_t* payload, size_t len, uint8_t* out) {
      uint8_t buf[kMaxPayload + kCrcSize];
      memcpy(buf, payload, len);
      const uint16_t crc = crc16(buf, len);
      buf[len] = (uint8_t)(crc & 0xFF);
      buf[len + 1] = (uint8_t)(crc >> 8);
      const size_t n = cobsEncode(buf, len + kCrcSize, out);
      out[n] = kDelimiter;
      return n + 1;
}

OpenResult openPacket(const uint8_t* packet, size_t len, uint8_t wantType, size_t wantLen, uint8_t* payload) {
      uint8_t buf[kMaxPayload + kCrcSize];
      size_t n = 0;
      if (!cobsDecode(packet, len, buf, sizeof(buf), &n) || n != wantLen + kCrcSize) {
            return OpenResult::kBadForm;
      }
      const uint16_t want = (uint16_t)buf[wantLen] | (uint16_t)((uint16_t)buf[wantLen + 1] << 8);
      if (crc16(buf, wantLen) != want) {
            return OpenResult::kBadCrc;
      }
      if (buf[0] != wantType) {
            return OpenResult::kBadForm;
      }
      memcpy(payload, buf, wantLen);
      return OpenResult::kOk;
}

static void putInt16(uint8_t* dst, int16_t v) {
      dst[0] = (uint8_t)((uint16_t)v & 0xFF);
      dst[1] = (uint8_t)((uint16_t)v >> 8);
}

void packTelemetry(const Telemetry& t, uint8_t* payload) {
      payload[0] = kTypeTelemetry;
      payload[1] = t.seq;
      payload[2] = t.batteryDeciVolts;
      payload[3] = t.status;
      payload[4] = t.capValEstimate;
      for (int i = 0; i < 4; i++) {
            putInt16(&payload[5 + i * 2], t.wheel[i]);
      }
      putInt16(&payload[13], t.accelX);
      putInt16(&payload[15], t.accelY);
      putInt16(&payload[17], t.yawRate);
      putInt16(&payload[19], t.yaw);
      payload[21] = t.rxBad;
}

int16_t scaleToInt16(float v, float scale) {
      if (isnan(v)) {
            return 0;
      }
      const float scaled = roundf(v * scale);
      if (scaled >= 32767.0f) {
            return 32767;
      }
      if (scaled <= -32768.0f) {
            return -32768;
      }
      return (int16_t)scaled;
}

}  // namespace ras
