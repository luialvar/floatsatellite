#pragma once

#include <Arduino.h>

#include "uart_link.h"

void yawControlBegin(float control_hz);
void yawControlSetGains(float kp, float ki);
void yawControlSetMaxRate(float max_rate_rad_s);
void yawControlReset();
float yawControlStep(float target_z, ZCommandMode mode, float yaw_measured_rad, float dt_s);
