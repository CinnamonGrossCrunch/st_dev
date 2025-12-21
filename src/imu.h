#pragma once
// imu.h - IMU + Magnetometer abstraction for Adafruit Feather nRF52840 Sense.
// Handles LSM6DS-family auto-detection and optional LIS3MDL magnetometer.

#include <Arduino.h>

namespace imu {

// 3D vector for compact BLE transmission.
struct Vec3 {
  float x, y, z;
};

// Sensor reading struct (accel in g, gyro in deg/s, mag in µT, temp in °C).
struct Reading {
  float acc_x, acc_y, acc_z;   // g
  float gyro_x, gyro_y, gyro_z; // deg/s
  float temp;                   // °C
  float mag_x, mag_y, mag_z;    // µT (valid only if magOk)
};

// Structured data for BLE (same values, grouped as Vec3).
struct Data {
  Vec3 accel_g;
  Vec3 gyro_dps;
  Vec3 mag_uT;
  float temp_C;
  bool magValid;
};

// Convert Reading to Data.
inline Data toData(const Reading& r, bool magValid) {
  Data d;
  d.accel_g = {r.acc_x, r.acc_y, r.acc_z};
  d.gyro_dps = {r.gyro_x, r.gyro_y, r.gyro_z};
  d.mag_uT = {r.mag_x, r.mag_y, r.mag_z};
  d.temp_C = r.temp;
  d.magValid = magValid;
  return d;
}

// Call once in setup() after Wire.begin().
// Returns true if at least the IMU initialized successfully.
bool begin();

// True if IMU is available.
bool imuOk();

// True if magnetometer is available.
bool magOk();

// Detected IMU model name (e.g. "LSM6DS3TRC"), or nullptr if not detected.
const char *imuModelName();

// Smoothing configuration (alpha: 0=max smooth, 1=no filter)
struct SmoothingConfig {
  float accel = 0.25f;  // Accelerometer (faster response)
  float gyro = 0.15f;   // Gyroscope (smooth out vibration)
  float mag = 0.08f;    // Magnetometer (slowest, changes gradually)
};

// Enable/disable smoothing (default: enabled with preset values)
void setSmoothing(bool enable, const SmoothingConfig& config = SmoothingConfig{});

// Read current sensor values. Returns false if IMU is not ready.
bool read(Reading &out);

}  // namespace imu
