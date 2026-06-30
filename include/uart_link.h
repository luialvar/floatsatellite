#pragma once

#include <Arduino.h>

#include "floatsat_config.h"

enum class ZCommandMode : uint8_t {
  kAttitude = 0,
  kRate = 1,
};

struct TelemetrySample {
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  float roll_rate_rad_s;
  float pitch_rate_rad_s;
  float yaw_rate_rad_s;
};

void uartBegin(uint32_t baud);
void uartUpdate();
bool uartTakeTarget(float &roll_rad, float &pitch_rad, float &z_rad_or_rad_s, ZCommandMode &z_mode);
void uartSendTelemetry(const TelemetrySample &sample);
