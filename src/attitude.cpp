#include "attitude.h"

#include "floatsat_config.h"

Attitude::Attitude()
    : imu_ready_(false),
      mag_calibrated_(false),
      roll_rad_(0.0f),
      pitch_rad_(0.0f),
      yaw_rad_(0.0f),
      roll_rate_rad_s_(0.0f),
      pitch_rate_rad_s_(0.0f),
      yaw_rate_rad_s_(0.0f),
      gyro_window_index_(0) {
  for (uint8_t i = 0; i < 3; ++i) {
    mag_min_[i] = 0.0f;
    mag_max_[i] = 0.0f;
    mag_offset_[i] = 0.0f;
    mag_scale_[i] = 1.0f;
  }

  for (uint8_t i = 0; i < 8; ++i) {
    gyro_window_x_[i] = 0.0f;
    gyro_window_y_[i] = 0.0f;
    gyro_window_z_[i] = 0.0f;
  }
}

bool Attitude::begin(float sample_hz) {
  filter_.begin(sample_hz);

  Wire.setSCL(PB10);
  Wire.setSDA(PB3);
  Wire.begin();
  Wire.setClock(400000);

  imu_ready_ = imu_.begin();
  return imu_ready_;
}

void Attitude::calibrateMagnetometer(uint32_t duration_ms) {
  if (!imu_ready_) return;

  mag_min_[0] = mag_min_[1] = mag_min_[2] = 1.0e9f;
  mag_max_[0] = mag_max_[1] = mag_max_[2] = -1.0e9f;

  const uint32_t start_ms = millis();
  while (millis() - start_ms < duration_ms) {
    imu_.readMag();

    const float values[3] = {
        imu_.calcMag(imu_.mx),
        imu_.calcMag(imu_.my),
        imu_.calcMag(imu_.mz),
    };

    for (uint8_t i = 0; i < 3; ++i) {
      if (values[i] < mag_min_[i]) mag_min_[i] = values[i];
      if (values[i] > mag_max_[i]) mag_max_[i] = values[i];
    }

    delay(10);
  }

  for (uint8_t i = 0; i < 3; ++i) {
    mag_offset_[i] = 0.5f * (mag_max_[i] + mag_min_[i]);
    mag_scale_[i] = 0.5f * (mag_max_[i] - mag_min_[i]);
    if (mag_scale_[i] < 1.0e-6f) mag_scale_[i] = 1.0f;
  }

  mag_calibrated_ = true;
}

bool Attitude::update() {
  if (!imu_ready_) return false;

  imu_.readGyro();
  imu_.readAccel();
  imu_.readMag();

  float mag_x = imu_.calcMag(imu_.mx);
  float mag_y = imu_.calcMag(imu_.my);
  float mag_z = imu_.calcMag(imu_.mz);

  if (mag_calibrated_) {
    mag_x = (mag_x - mag_offset_[0]) / mag_scale_[0];
    mag_y = (mag_y - mag_offset_[1]) / mag_scale_[1];
    mag_z = (mag_z - mag_offset_[2]) / mag_scale_[2];
  }

  const float gyro_x_dps = imu_.calcGyro(-imu_.gx);
  const float gyro_y_dps = imu_.calcGyro(-imu_.gy);
  const float gyro_z_dps = imu_.calcGyro(-imu_.gz);

  filter_.update(
      gyro_x_dps,
      gyro_y_dps,
      gyro_z_dps,
      imu_.calcAccel(imu_.ax),
      imu_.calcAccel(imu_.ay),
      imu_.calcAccel(imu_.az),
      -mag_x,
      mag_y,
      mag_z);

  roll_rad_ = filter_.getRoll() * floatsat::kDegToRad;
  pitch_rad_ = filter_.getPitch() * floatsat::kDegToRad;
  yaw_rad_ = filter_.getYaw() * floatsat::kDegToRad;

  applyMountYawCorrection();
  updateAverageRates(gyro_x_dps, gyro_y_dps, gyro_z_dps);
  return true;
}

void Attitude::updateAverageRates(float gx_dps, float gy_dps, float gz_dps) {
  gyro_window_x_[gyro_window_index_] = gx_dps;
  gyro_window_y_[gyro_window_index_] = gy_dps;
  gyro_window_z_[gyro_window_index_] = gz_dps;
  gyro_window_index_ = (gyro_window_index_ + 1) % 8;

  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;

  for (uint8_t i = 0; i < 8; ++i) {
    sum_x += gyro_window_x_[i];
    sum_y += gyro_window_y_[i];
    sum_z += gyro_window_z_[i];
  }

  roll_rate_rad_s_ = -(sum_x / 8.0f) * floatsat::kDegToRad;
  pitch_rate_rad_s_ = -(sum_y / 8.0f) * floatsat::kDegToRad;
  yaw_rate_rad_s_ = -(sum_z / 8.0f) * floatsat::kDegToRad;
}

void Attitude::applyMountYawCorrection() {
  float attitude_matrix[3][3];
  yprToMatrix(yaw_rad_, pitch_rad_, roll_rad_, attitude_matrix);

  float mount_matrix[3][3];
  yawRotation(floatsat::kImuMountYawOffsetRad, mount_matrix);

  float corrected[3][3];
  multiply3x3(attitude_matrix, mount_matrix, corrected);

  matrixToYpr(corrected, &yaw_rad_, &pitch_rad_, &roll_rad_);
  yaw_rad_ = floatsat::wrapTwoPi(yaw_rad_);
}

void Attitude::yprToMatrix(float yaw, float pitch, float roll, float out[3][3]) {
  const float cy = cosf(yaw);
  const float sy = sinf(yaw);
  const float cp = cosf(pitch);
  const float sp = sinf(pitch);
  const float cr = cosf(roll);
  const float sr = sinf(roll);

  out[0][0] = cy * cp;
  out[0][1] = cy * sp * sr - sy * cr;
  out[0][2] = cy * sp * cr + sy * sr;

  out[1][0] = sy * cp;
  out[1][1] = sy * sp * sr + cy * cr;
  out[1][2] = sy * sp * cr - cy * sr;

  out[2][0] = -sp;
  out[2][1] = cp * sr;
  out[2][2] = cp * cr;
}

void Attitude::matrixToYpr(const float matrix[3][3], float *yaw, float *pitch, float *roll) {
  *pitch = asinf(-matrix[2][0]);
  *roll = atan2f(matrix[2][1], matrix[2][2]);
  *yaw = atan2f(matrix[1][0], matrix[0][0]);
}

void Attitude::multiply3x3(const float a[3][3], const float b[3][3], float out[3][3]) {
  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      out[row][col] =
          a[row][0] * b[0][col] +
          a[row][1] * b[1][col] +
          a[row][2] * b[2][col];
    }
  }
}

void Attitude::yawRotation(float yaw, float out[3][3]) {
  const float c = cosf(yaw);
  const float s = sinf(yaw);

  out[0][0] = c;
  out[0][1] = -s;
  out[0][2] = 0.0f;
  out[1][0] = s;
  out[1][1] = c;
  out[1][2] = 0.0f;
  out[2][0] = 0.0f;
  out[2][1] = 0.0f;
  out[2][2] = 1.0f;
}
