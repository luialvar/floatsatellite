#include "yaw_control.h"

#include "floatsat_config.h"

namespace {

float kp = 0.2f;
float ki = 0.0f;
float max_rate_rad_s = 1.2f;
float integral = 0.0f;
float default_dt_s = 0.02f;
ZCommandMode last_mode = ZCommandMode::kAttitude;
bool mode_initialized = false;

}  // namespace

void yawControlBegin(float control_hz) {
  if (control_hz > 1.0f && isfinite(control_hz)) {
    default_dt_s = 1.0f / control_hz;
  } else {
    default_dt_s = 0.02f;
  }

  kp = 0.2f;
  ki = 0.0f;
  max_rate_rad_s = 1.2f;
  yawControlReset();
  mode_initialized = false;
}

void yawControlSetGains(float new_kp, float new_ki) {
  kp = new_kp;
  ki = new_ki;
}

void yawControlSetMaxRate(float new_max_rate_rad_s) {
  max_rate_rad_s = (new_max_rate_rad_s < 0.01f) ? 0.01f : new_max_rate_rad_s;
}

void yawControlReset() {
  integral = 0.0f;
}

float yawControlStep(float target_z, ZCommandMode mode, float yaw_measured_rad, float dt_s) {
  if (!(dt_s > 0.0f) || !isfinite(dt_s) || dt_s > 0.2f) {
    dt_s = default_dt_s;
  }

  if (!mode_initialized || mode != last_mode) {
    yawControlReset();
    last_mode = mode;
    mode_initialized = true;
  }

  if (mode == ZCommandMode::kRate) {
    return floatsat::clamp(target_z, -max_rate_rad_s, max_rate_rad_s);
  }

  const float error = floatsat::wrapPi(target_z - yaw_measured_rad);
  const float proposed_integral = integral + error * dt_s;
  const float unsaturated = kp * error + ki * proposed_integral;
  const float command = floatsat::clamp(unsaturated, -max_rate_rad_s, max_rate_rad_s);

  const bool saturated_high = unsaturated > max_rate_rad_s;
  const bool saturated_low = unsaturated < -max_rate_rad_s;
  const bool pushing_further =
      (saturated_high && error > 0.0f) ||
      (saturated_low && error < 0.0f);

  if (!pushing_further) integral = proposed_integral;

  return command;
}
