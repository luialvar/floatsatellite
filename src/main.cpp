#include <Arduino.h>
#include <HardwareTimer.h>

#include "attitude.h"
#include "floatsat_config.h"
#include "reaction_wheels.h"
#include "uart_link.h"
#include "yaw_control.h"

namespace {

constexpr uint32_t kAttitudeDiv = floatsat::kBaseTickHz / floatsat::kAttitudeUpdateHz;
constexpr uint32_t kControlDiv = floatsat::kBaseTickHz / floatsat::kControlUpdateHz;
constexpr uint32_t kWheelRampDiv = floatsat::kBaseTickHz / floatsat::kWheelRampUpdateHz;
constexpr uint32_t kTelemetryDiv = floatsat::kBaseTickHz / floatsat::kTelemetryUpdateHz;

struct Targets {
  float roll_rad = 0.0f;
  float pitch_rad = 0.0f;
  float z_rad_or_rad_s = 0.0f;
  ZCommandMode z_mode = ZCommandMode::kAttitude;
};

struct AxisPiController {
  float kp;
  float ki;
  float max_rate_rad_s;
  float integral;

  float step(float target_rad, float measured_rad, float dt_s) {
    const float error = floatsat::wrapPi(target_rad - measured_rad);
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
};

HardwareTimer *system_timer = nullptr;
Attitude attitude;
Targets targets;

AxisPiController roll_controller{0.25f, 0.0f, 1.0f, 0.0f};
AxisPiController pitch_controller{0.25f, 0.0f, 1.0f, 0.0f};

volatile bool attitude_update_flag = false;
volatile bool control_update_flag = false;
volatile bool wheel_ramp_update_flag = false;
volatile bool telemetry_update_flag = false;

void systemTimerCallback() {
  static uint32_t attitude_count = 0;
  static uint32_t control_count = 0;
  static uint32_t ramp_count = 0;
  static uint32_t telemetry_count = 0;

  if (++attitude_count >= kAttitudeDiv) {
    attitude_count = 0;
    attitude_update_flag = true;
  }

  if (++control_count >= kControlDiv) {
    control_count = 0;
    control_update_flag = true;
  }

  if (++ramp_count >= kWheelRampDiv) {
    ramp_count = 0;
    wheel_ramp_update_flag = true;
  }

  if (++telemetry_count >= kTelemetryDiv) {
    telemetry_count = 0;
    telemetry_update_flag = true;
  }
}

void beginTimer() {
  system_timer = new HardwareTimer(TIM2);
  system_timer->setOverflow(floatsat::kBaseTickHz, HERTZ_FORMAT);
  system_timer->attachInterrupt(systemTimerCallback);
  system_timer->resume();
}

void updateTargetsFromUart() {
  float roll = 0.0f;
  float pitch = 0.0f;
  float z = 0.0f;
  ZCommandMode z_mode = ZCommandMode::kAttitude;

  if (uartTakeTarget(roll, pitch, z, z_mode)) {
    targets.roll_rad = roll;
    targets.pitch_rad = pitch;
    targets.z_rad_or_rad_s = z;
    targets.z_mode = z_mode;
  }
}

void runControlStep() {
  const float dt_s = 1.0f / static_cast<float>(floatsat::kControlUpdateHz);

  const float roll_rate_cmd =
      roll_controller.step(targets.roll_rad, attitude.roll(), dt_s);
  const float pitch_rate_cmd =
      pitch_controller.step(targets.pitch_rad, attitude.pitch(), dt_s);
  const float yaw_rate_cmd =
      yawControlStep(targets.z_rad_or_rad_s, targets.z_mode, attitude.yaw(), dt_s);

  reactionWheelsSetBodyRates(roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd);
}

void sendTelemetry() {
  const TelemetrySample sample{
      attitude.roll(),
      attitude.pitch(),
      attitude.yaw(),
      attitude.rollRate(),
      attitude.pitchRate(),
      attitude.yawRate(),
  };

  uartSendTelemetry(sample);

  Serial.print(sample.yaw_rad, 6);
  Serial.print(',');
  Serial.print(sample.pitch_rad, 6);
  Serial.print(',');
  Serial.println(sample.roll_rad, 6);
}

}  // namespace

void setup() {
  Serial.begin(floatsat::kDebugBaud);
  delay(300);

  Serial.println("FloatSat embedded firmware starting.");
  uartBegin(floatsat::kLinkBaud);
  reactionWheelsBegin();

  if (!attitude.begin(static_cast<float>(floatsat::kAttitudeUpdateHz))) {
    Serial.println("ERROR: LSM9DS1 IMU not detected. Motors disabled.");
    reactionWheelsStop();
    while (true) {
      uartUpdate();
      delay(100);
    }
  }

  Serial.println("Rotate the satellite during magnetometer calibration.");
  attitude.calibrateMagnetometer(floatsat::kMagCalibrationMs);
  Serial.println("Magnetometer calibration complete.");

  yawControlBegin(static_cast<float>(floatsat::kControlUpdateHz));
  beginTimer();

  Serial.println("FloatSat firmware ready.");
}

void loop() {
  uartUpdate();
  updateTargetsFromUart();

  if (attitude_update_flag) {
    attitude_update_flag = false;
    attitude.update();
  }

  if (control_update_flag) {
    control_update_flag = false;
    runControlStep();
  }

  if (wheel_ramp_update_flag) {
    wheel_ramp_update_flag = false;
    reactionWheelsRampUpdate(1.0f / static_cast<float>(floatsat::kWheelRampUpdateHz));
  }

  if (telemetry_update_flag) {
    telemetry_update_flag = false;
    sendTelemetry();
  }

  reactionWheelsTick();
}
