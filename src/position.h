#pragma once
// position.h - Relative position tracking via IMU integration
// WARNING: Position will drift over time due to sensor noise and bias.
// Best used for short-term tracking (<10s) or with periodic re-taring.

#include <Arduino.h>

namespace position {

// 3D position vector (meters from tare point)
struct Vec3 {
  float x, y, z;
};

// Configuration for drift reduction
struct Config {
  // Stationary detection
  float stationaryVarianceThreshold = 0.001f;  // g² - accel variance below this = stationary (tighter = less false positives)
  
  // Dead zone - ignore tiny accelerations (sensor noise)
  float accelDeadZone = 0.02f;  // g - accelerations below this are zeroed
  
  // Velocity decay - slowly reduce velocity when acceleration is small
  float velocityDecayRate = 0.98f;  // per update (0.98 = 2% decay per 20ms) - less aggressive
  float velocityDecayThreshold = 0.03f;  // g - decay velocity when accel magnitude below this
  
  // ZUPT (Zero Velocity Update)
  bool enableZeroVelocityUpdate = true;
  float zeroVelocityThreshold = 0.05f;  // m/s - force to zero when stationary and below this
  
  // Position decay - pull position toward zero when stationary
  // This is for LONG-TERM drift correction, not during a lift!
  bool enablePositionDecay = true;
  float positionDecayRate = 0.998f;  // per update when stationary (0.998 = 0.2% per 20ms, ~10% per second)
  float positionDecayDelay = 1.0f;   // seconds - wait this long before starting position decay
};

// Initialize position tracking (call after imu::begin())
void begin(const Config& cfg = Config{});

// Set current position as origin (resets position and velocity to zero)
void tare();

// Update position based on latest IMU reading (call at fixed rate, e.g. 50Hz)
// Pass raw accelerometer data in g's (from imu::Reading)
// Returns time delta used for integration (ms)
uint32_t update(float acc_x_g, float acc_y_g, float acc_z_g);

// Get current position relative to tare point (meters)
Vec3 getPosition();

// Get current velocity (m/s)
Vec3 getVelocity();

// Get acceleration with gravity removed (m/s²)
Vec3 getAccelWorldFrame();

// Check if device is currently stationary (for debugging)
bool isStationary();

// Reset position and velocity without changing tare reference
void resetPosition();

}  // namespace position
