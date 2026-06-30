#pragma once

#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>

class Attitude {
 public:
  Attitude();

  bool begin(float sample_hz);
  void calibrateMagnetometer(uint32_t duration_ms);
  bool update();

  float roll() const { return roll_rad_; }
  float pitch() const { return pitch_rad_; }
  float yaw() const { return yaw_rad_; }

  float rollRate() const { return roll_rate_rad_s_; }
  float pitchRate() const { return pitch_rate_rad_s_; }
  float yawRate() const { return yaw_rate_rad_s_; }

 private:
  void updateAverageRates(float gx_dps, float gy_dps, float gz_dps);
  void applyMountYawCorrection();

  static void yprToMatrix(float yaw, float pitch, float roll, float out[3][3]);
  static void matrixToYpr(const float matrix[3][3], float *yaw, float *pitch, float *roll);
  static void multiply3x3(const float a[3][3], const float b[3][3], float out[3][3]);
  static void yawRotation(float yaw, float out[3][3]);

  Madgwick filter_;
  LSM9DS1 imu_;

  bool imu_ready_;
  bool mag_calibrated_;

  float roll_rad_;
  float pitch_rad_;
  float yaw_rad_;

  float roll_rate_rad_s_;
  float pitch_rate_rad_s_;
  float yaw_rate_rad_s_;

  float mag_min_[3];
  float mag_max_[3];
  float mag_offset_[3];
  float mag_scale_[3];

  float gyro_window_x_[8];
  float gyro_window_y_[8];
  float gyro_window_z_[8];
  uint8_t gyro_window_index_;
};
