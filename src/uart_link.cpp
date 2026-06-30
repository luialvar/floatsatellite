#include "uart_link.h"

#include <math.h>

namespace {

HardwareSerial Serial2(PA3, PA2);
HardwareSerial &link = Serial2;

constexpr uint8_t kSof1 = 0xAA;
constexpr uint8_t kSof2 = 0x55;
constexpr uint8_t kCommandPayloadLen = 3;
constexpr uint8_t kTelemetryPayloadLen = floatsat::kTelemetryValues * 2;

constexpr int32_t kScale = 100;
constexpr int32_t kRaw180 = 18000;
constexpr uint16_t kRaw360 = 36000;

enum class RxState : uint8_t {
  kWaitSof1,
  kWaitSof2,
  kPayload,
  kCrcLo,
  kCrcHi,
};

RxState rx_state = RxState::kWaitSof1;
uint8_t rx_payload[kCommandPayloadLen];
uint8_t rx_index = 0;
uint8_t rx_crc_lo = 0;
uint16_t rx_crc_calc = 0xFFFF;

bool has_new_target = false;
float target_roll_rad = 0.0f;
float target_pitch_rad = 0.0f;
float target_z = 0.0f;
ZCommandMode target_z_mode = ZCommandMode::kAttitude;

uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data) << 8));
  for (uint8_t bit = 0; bit < 8; ++bit) {
    const bool msb_set = (crc & 0x8000) != 0;
    crc = static_cast<uint16_t>(crc << 1);
    if (msb_set) crc = static_cast<uint16_t>(crc ^ 0x1021);
  }
  return crc;
}

uint16_t crc16Calc(const uint8_t *payload, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; ++i) crc = crc16Update(crc, payload[i]);
  return crc;
}

void putU16Le(uint8_t *dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>(value >> 8);
}

uint16_t getU16Le(const uint8_t *src) {
  return static_cast<uint16_t>(src[0]) | (static_cast<uint16_t>(src[1]) << 8);
}

uint16_t encodeAngleRad(float angle_rad) {
  const float deg = floatsat::wrapTwoPi(angle_rad) * floatsat::kRadToDeg;
  const long raw = lroundf(deg * static_cast<float>(kScale));
  if (raw < 0) return 0;
  if (raw > kRaw360) return kRaw360;
  return static_cast<uint16_t>(raw);
}

uint16_t encodeSignedDeg(float value_deg) {
  value_deg = floatsat::clamp(value_deg, -180.0f, 180.0f);

  long raw = 0;
  if (value_deg >= 0.0f) {
    raw = lroundf(value_deg * static_cast<float>(kScale));
  } else {
    raw = kRaw180 - lroundf(value_deg * static_cast<float>(kScale));
  }

  if (raw < 0) return 0;
  if (raw > kRaw360) return kRaw360;
  return static_cast<uint16_t>(raw);
}

float decodeUnsignedAngleRad(uint16_t raw) {
  if (raw > kRaw360) raw = kRaw360;
  return (static_cast<float>(raw) / static_cast<float>(kScale)) * floatsat::kDegToRad;
}

float decodeSignedDeg(uint16_t raw) {
  if (raw > kRaw360) raw = kRaw360;

  int32_t signed_raw = static_cast<int32_t>(raw);
  if (signed_raw > kRaw180) signed_raw = kRaw180 - signed_raw;

  return static_cast<float>(signed_raw) / static_cast<float>(kScale);
}

void updateMailbox(uint8_t type, uint16_t raw) {
  switch (type) {
    case 0:
      target_roll_rad = decodeUnsignedAngleRad(raw);
      break;
    case 1:
      target_pitch_rad = decodeUnsignedAngleRad(raw);
      break;
    case 2:
      target_z = decodeUnsignedAngleRad(raw);
      target_z_mode = ZCommandMode::kAttitude;
      break;
    case 3:
      target_z = decodeSignedDeg(raw) * floatsat::kDegToRad;
      target_z_mode = ZCommandMode::kRate;
      break;
    default:
      return;
  }

  has_new_target = true;
}

}  // namespace

void uartBegin(uint32_t baud) {
  link.begin(baud);
}

void uartUpdate() {
  while (link.available()) {
    const uint8_t byte = static_cast<uint8_t>(link.read());

    switch (rx_state) {
      case RxState::kWaitSof1:
        rx_state = (byte == kSof1) ? RxState::kWaitSof2 : RxState::kWaitSof1;
        break;

      case RxState::kWaitSof2:
        if (byte == kSof2) {
          rx_index = 0;
          rx_crc_calc = 0xFFFF;
          rx_state = RxState::kPayload;
        } else {
          rx_state = RxState::kWaitSof1;
        }
        break;

      case RxState::kPayload:
        rx_payload[rx_index++] = byte;
        rx_crc_calc = crc16Update(rx_crc_calc, byte);
        if (rx_index >= kCommandPayloadLen) rx_state = RxState::kCrcLo;
        break;

      case RxState::kCrcLo:
        rx_crc_lo = byte;
        rx_state = RxState::kCrcHi;
        break;

      case RxState::kCrcHi: {
        const uint16_t received_crc =
            static_cast<uint16_t>(rx_crc_lo) | (static_cast<uint16_t>(byte) << 8);
        rx_state = RxState::kWaitSof1;

        if (received_crc != rx_crc_calc) break;

        const uint8_t type = rx_payload[0];
        if (type > 3) break;

        updateMailbox(type, getU16Le(&rx_payload[1]));
        break;
      }
    }
  }
}

bool uartTakeTarget(float &roll_rad, float &pitch_rad, float &z_rad_or_rad_s, ZCommandMode &z_mode) {
  if (!has_new_target) return false;

  has_new_target = false;
  roll_rad = target_roll_rad;
  pitch_rad = target_pitch_rad;
  z_rad_or_rad_s = target_z;
  z_mode = target_z_mode;
  return true;
}

void uartSendTelemetry(const TelemetrySample &sample) {
  uint8_t payload[kTelemetryPayloadLen];

  putU16Le(&payload[0], encodeAngleRad(sample.roll_rad));
  putU16Le(&payload[2], encodeAngleRad(sample.pitch_rad));
  putU16Le(&payload[4], encodeAngleRad(sample.yaw_rad));
  putU16Le(&payload[6], encodeSignedDeg(sample.roll_rate_rad_s * floatsat::kRadToDeg));
  putU16Le(&payload[8], encodeSignedDeg(sample.pitch_rate_rad_s * floatsat::kRadToDeg));
  putU16Le(&payload[10], encodeSignedDeg(sample.yaw_rate_rad_s * floatsat::kRadToDeg));

  const uint16_t crc = crc16Calc(payload, kTelemetryPayloadLen);

  link.write(kSof1);
  link.write(kSof2);
  link.write(payload, kTelemetryPayloadLen);
  link.write(static_cast<uint8_t>(crc & 0xFF));
  link.write(static_cast<uint8_t>(crc >> 8));
}
