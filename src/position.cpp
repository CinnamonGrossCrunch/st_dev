// position.cpp - Relative position tracking implementation
#include "position.h"

namespace position {

// ---- State variables ----
static Config s_config;
static bool s_initialized = false;

// Tare reference (initial gravity vector in sensor frame)
static Vec3 s_gravityRef = {0.0f, 0.0f, 1.0f};  // Default: Z-up, 1g

// Current state
static Vec3 s_position = {0.0f, 0.0f, 0.0f};    // meters
static Vec3 s_velocity = {0.0f, 0.0f, 0.0f};    // m/s
static Vec3 s_accelWorld = {0.0f, 0.0f, 0.0f};  // m/s² (gravity removed)

// Timing
static uint32_t s_lastUpdateMs = 0;

// Stationary detection (simple variance check)
static float s_accelHistory[10] = {0};
static uint8_t s_accelHistoryIdx = 0;
static bool s_isStationary = false;
static uint32_t s_stationaryStartMs = 0;  // When we first became stationary

// Flag to recapture gravity reference on next update
static bool s_needsGravityCapture = true;

// ---- Helpers ----

static float magnitude(const Vec3& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static float variance(const float* samples, uint8_t count) {
  float mean = 0.0f;
  for (uint8_t i = 0; i < count; i++) mean += samples[i];
  mean /= count;
  
  float var = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    float diff = samples[i] - mean;
    var += diff * diff;
  }
  return var / count;
}

// ---- Public API ----

void begin(const Config& cfg) {
  s_config = cfg;
  s_initialized = true;
  s_lastUpdateMs = millis();
}

void tare() {
  // Capture current acceleration as gravity reference
  // (assumes device is stationary during tare)
  // This is set by the next update() call
  
  s_position = {0.0f, 0.0f, 0.0f};
  s_velocity = {0.0f, 0.0f, 0.0f};
  s_accelWorld = {0.0f, 0.0f, 0.0f};
  s_lastUpdateMs = millis();
  
  // Clear history
  for (uint8_t i = 0; i < 10; i++) s_accelHistory[i] = 0.0f;
  s_accelHistoryIdx = 0;
  
  // Flag to recapture gravity on next update
  s_needsGravityCapture = true;
}

void resetPosition() {
  s_position = {0.0f, 0.0f, 0.0f};
}

uint32_t update(float acc_x_g, float acc_y_g, float acc_z_g) {
  if (!s_initialized) {
    begin();
  }
  
  // Capture gravity reference when needed (first call or after tare)
  if (s_needsGravityCapture) {
    // Store gravity vector and compute its magnitude for normalization
    float grav_mag = sqrtf(acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g);
    if (grav_mag > 0.5f) {  // Sanity check - should be ~1g
      // Store normalized gravity direction (points "down" in sensor frame)
      s_gravityRef.x = acc_x_g / grav_mag;
      s_gravityRef.y = acc_y_g / grav_mag;
      s_gravityRef.z = acc_z_g / grav_mag;
    }
    s_lastUpdateMs = millis();
    s_needsGravityCapture = false;
    return 0;  // Skip integration on gravity capture
  }
  
  uint32_t now = millis();
  uint32_t dt_ms = now - s_lastUpdateMs;
  
  // Limit dt to prevent huge jumps after delays
  if (dt_ms > 100 || dt_ms == 0) {
    s_lastUpdateMs = now;
    return 0;  // Skip this update, wait for next valid sample
  }
  
  float dt_s = dt_ms / 1000.0f;
  
  // Project acceleration onto gravity axis
  // Dot product of accel with gravity direction gives vertical component
  // Positive = upward (against gravity), Negative = downward (with gravity)
  float accel_vertical_g = (acc_x_g * s_gravityRef.x + 
                            acc_y_g * s_gravityRef.y + 
                            acc_z_g * s_gravityRef.z);
  
  // Remove gravity (1g in the "down" direction)
  // After removal, positive = accelerating upward, negative = accelerating downward
  float accel_vertical_net_g = accel_vertical_g - 1.0f;
  
  // Convert to m/s²
  const float G = 9.80665f;
  float accel_vertical_ms2 = accel_vertical_net_g * G;
  
  // Store in world frame (Z = vertical, X/Y unused for now)
  s_accelWorld.x = 0.0f;
  s_accelWorld.y = 0.0f;
  s_accelWorld.z = accel_vertical_ms2;  // positive Z = up
  
  // Apply dead zone - zero out tiny accelerations (likely noise/bias)
  float accelWorld_mag = fabsf(accel_vertical_net_g);
  if (accelWorld_mag < s_config.accelDeadZone) {
    s_accelWorld.z = 0.0f;
    accelWorld_mag = 0.0f;
  }
  
  // Stationary detection: track acceleration magnitude variance
  float accel_mag = sqrtf(acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g);
  s_accelHistory[s_accelHistoryIdx] = accel_mag;
  s_accelHistoryIdx = (s_accelHistoryIdx + 1) % 10;
  
  float accel_var = variance(s_accelHistory, 10);
  bool wasStationary = s_isStationary;
  s_isStationary = (accel_var < s_config.stationaryVarianceThreshold);
  
  // Track when we first became stationary
  if (s_isStationary && !wasStationary) {
    s_stationaryStartMs = now;
  }
  
  // Calculate how long we've been stationary
  float stationaryDuration = s_isStationary ? (now - s_stationaryStartMs) / 1000.0f : 0.0f;
  
  // Zero-velocity update (ZUPT): reset velocity when stationary
  if (s_config.enableZeroVelocityUpdate && s_isStationary) {
    if (fabsf(s_velocity.z) < s_config.zeroVelocityThreshold) {
      s_velocity.z = 0.0f;
    }
  }
  
  // Position decay: pull position toward zero when stationary for long enough
  // This corrects for accumulated drift when device returns to rest
  // Only start decaying after the delay period (don't decay during pauses at zenith!)
  if (s_config.enablePositionDecay && s_isStationary && 
      stationaryDuration > s_config.positionDecayDelay) {
    s_position.z *= s_config.positionDecayRate;
  }
  
  // Velocity decay - reduce velocity when acceleration is small
  // This counteracts drift from bias/noise
  if (accelWorld_mag < s_config.velocityDecayThreshold) {
    s_velocity.z *= s_config.velocityDecayRate;
  }
  
  // Integrate acceleration to velocity (vertical only)
  s_velocity.z += s_accelWorld.z * dt_s;
  
  // Integrate velocity to position (vertical only)
  s_position.z += s_velocity.z * dt_s;
  
  s_lastUpdateMs = now;
  return dt_ms;
}

Vec3 getPosition() {
  return s_position;
}

Vec3 getVelocity() {
  return s_velocity;
}

Vec3 getAccelWorldFrame() {
  return s_accelWorld;
}

bool isStationary() {
  return s_isStationary;
}

}  // namespace position
