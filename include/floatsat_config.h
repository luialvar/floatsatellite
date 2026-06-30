#pragma once

#include <Arduino.h>
#include <math.h>

namespace floatsat {

constexpr uint32_t kDebugBaud = 115200;
constexpr uint32_t kLinkBaud = 115200;

constexpr uint32_t kBaseTickHz = 1000;
constexpr uint32_t kAttitudeUpdateHz = 200;
constexpr uint32_t kControlUpdateHz = 50;
constexpr uint32_t kWheelRampUpdateHz = 50;
constexpr uint32_t kTelemetryUpdateHz = 10;

constexpr uint8_t kTelemetryValues = 6;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;

constexpr float kReactionWheelSpeedScale = 40.0f;
constexpr float kWheelMaxSpeedRadS = 130.0f;
constexpr float kWheelAccelLimitRadS2 = 40.0f;
constexpr float kWheelMinVoltage = 7.0f;
constexpr float kWheelMaxVoltage = 12.0f;

constexpr float kImuMountYawOffsetRad = -kPi / 4.0f;
constexpr uint32_t kMagCalibrationMs = 15000;

inline float clamp(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

inline float wrapPi(float angle) {
  angle = fmodf(angle + kPi, kTwoPi);
  if (angle < 0.0f) angle += kTwoPi;
  return angle - kPi;
}

inline float wrapTwoPi(float angle) {
  angle = fmodf(angle, kTwoPi);
  if (angle < 0.0f) angle += kTwoPi;
  return angle;
}

}  // namespace floatsat
