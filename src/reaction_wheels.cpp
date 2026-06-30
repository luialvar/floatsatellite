#include "reaction_wheels.h"

#include <SimpleFOC.h>

#include "floatsat_config.h"

namespace {

BLDCMotor motor_x(7);
BLDCMotor motor_y(7);
BLDCMotor motor_z(7);

BLDCDriver3PWM driver_x(PA8, PA9, PA10, PB13);
BLDCDriver3PWM driver_y(PB0, PA5, PB4, PA15);
BLDCDriver3PWM driver_z(PB6, PB7, PB8, PB12);

float target_x_rad_s = 0.0f;
float target_y_rad_s = 0.0f;
float target_z_rad_s = 0.0f;

float current_x_rad_s = 0.0f;
float current_y_rad_s = 0.0f;
float current_z_rad_s = 0.0f;

float slewToward(float current, float target, float max_step) {
  const float delta = floatsat::clamp(target - current, -max_step, max_step);
  return current + delta;
}

float voltageForSpeed(float speed_rad_s) {
  const float abs_speed = fabsf(speed_rad_s);
  if (abs_speed < 0.01f) return 0.0f;

  const float ratio = floatsat::clamp(abs_speed / floatsat::kWheelMaxSpeedRadS, 0.0f, 1.0f);
  return floatsat::kWheelMinVoltage +
         (floatsat::kWheelMaxVoltage - floatsat::kWheelMinVoltage) * ratio;
}

void configureMotor(BLDCMotor &motor, BLDCDriver3PWM &driver) {
  driver.voltage_power_supply = 12.0f;
  driver.pwm_frequency = 40000;
  driver.init();

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;
  motor.phase_resistance = 14.4f;
  motor.KV_rating = 128.0f;
  motor.current_limit = 0.8f;
  motor.velocity_limit = 200.0f;
  motor.voltage_limit = 0.0f;
  motor.init();
  motor.initFOC();
}

}  // namespace

void reactionWheelsBegin() {
  configureMotor(motor_x, driver_x);
  configureMotor(motor_y, driver_y);
  configureMotor(motor_z, driver_z);
}

void reactionWheelsSetBodyRates(float roll_rate_rad_s, float pitch_rate_rad_s, float yaw_rate_rad_s) {
  target_x_rad_s = -roll_rate_rad_s * floatsat::kReactionWheelSpeedScale;
  target_y_rad_s = -pitch_rate_rad_s * floatsat::kReactionWheelSpeedScale;
  target_z_rad_s = -yaw_rate_rad_s * floatsat::kReactionWheelSpeedScale;

  target_x_rad_s = floatsat::clamp(target_x_rad_s, -floatsat::kWheelMaxSpeedRadS, floatsat::kWheelMaxSpeedRadS);
  target_y_rad_s = floatsat::clamp(target_y_rad_s, -floatsat::kWheelMaxSpeedRadS, floatsat::kWheelMaxSpeedRadS);
  target_z_rad_s = floatsat::clamp(target_z_rad_s, -floatsat::kWheelMaxSpeedRadS, floatsat::kWheelMaxSpeedRadS);
}

void reactionWheelsRampUpdate(float dt_s) {
  if (!(dt_s > 0.0f) || !isfinite(dt_s) || dt_s > 0.5f) {
    dt_s = 1.0f / floatsat::kWheelRampUpdateHz;
  }

  const float max_step = floatsat::kWheelAccelLimitRadS2 * dt_s;

  current_x_rad_s = slewToward(current_x_rad_s, target_x_rad_s, max_step);
  current_y_rad_s = slewToward(current_y_rad_s, target_y_rad_s, max_step);
  current_z_rad_s = slewToward(current_z_rad_s, target_z_rad_s, max_step);

  motor_x.voltage_limit = voltageForSpeed(current_x_rad_s);
  motor_y.voltage_limit = voltageForSpeed(current_y_rad_s);
  motor_z.voltage_limit = voltageForSpeed(current_z_rad_s);
}

void reactionWheelsTick() {
  motor_x.loopFOC();
  motor_y.loopFOC();
  motor_z.loopFOC();

  motor_x.move(current_x_rad_s);
  motor_y.move(current_y_rad_s);
  motor_z.move(current_z_rad_s);
}

void reactionWheelsStop() {
  target_x_rad_s = 0.0f;
  target_y_rad_s = 0.0f;
  target_z_rad_s = 0.0f;
  current_x_rad_s = 0.0f;
  current_y_rad_s = 0.0f;
  current_z_rad_s = 0.0f;

  motor_x.move(0.0f);
  motor_y.move(0.0f);
  motor_z.move(0.0f);
  delay(100);

  motor_x.disable();
  motor_y.disable();
  motor_z.disable();
}
