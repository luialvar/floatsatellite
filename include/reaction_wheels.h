#pragma once

#include <Arduino.h>

void reactionWheelsBegin();
void reactionWheelsSetBodyRates(float roll_rate_rad_s, float pitch_rate_rad_s, float yaw_rate_rad_s);
void reactionWheelsRampUpdate(float dt_s);
void reactionWheelsTick();
void reactionWheelsStop();
