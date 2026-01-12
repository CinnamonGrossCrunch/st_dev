#pragma once
// loadcell_backend.h - Abstract backend interface for load cell ADCs
// Supports HX711 and NAU7802 backends via compile-time selection

#include <Arduino.h>

namespace loadcell_backend {

// Backend configuration - passed to init()
// HX711 uses sckPin/dtPin, NAU7802 ignores them (uses I2C)
struct BackendConfig {
  uint8_t sckPin = 11;               // HX711 clock pin
  uint8_t dtPin = 12;                // HX711 data pin
  float calibrationFactor = 800.0f;  // Counts per pound
  int32_t zeroOffset = 0;            // Tare offset
};

// Initialize the backend hardware
// Returns true on success
bool init(const BackendConfig& cfg);

// Check if new data is ready (non-blocking)
bool isReady();

// Read raw ADC counts (blocking if not ready)
// Returns true if read successful
bool readRaw(int32_t& value);

// Read calibrated weight in pounds
// Returns true if read successful
bool read(float& weightLbs);

// Tare: set current load as zero
// samples: number of readings to average
bool tare(uint8_t samples = 10);

// Calibrate with known weight
// knownWeightLbs: weight of calibration mass
// samples: number of readings to average
bool calibrate(float knownWeightLbs, uint8_t samples = 10);

// Get/set calibration factor
void setCalibration(float factor);
float getCalibration();

// Get zero offset
int32_t getZeroOffset();

// Save calibration to persistent storage
bool saveCalibration();

// Load calibration from persistent storage
bool loadCalibration();

}  // namespace loadcell_backend
