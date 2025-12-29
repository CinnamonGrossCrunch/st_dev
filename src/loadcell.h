#pragma once
// loadcell.h - HX711 load cell interface
// Reads weight/force from load cell via HX711 ADC

#include <Arduino.h>

namespace loadcell {

// Configuration
struct Config {
  uint8_t sckPin = 11;   // Clock pin (D11)
  uint8_t dtPin = 12;    // Data pin (D12)
  float calibrationFactor = 800.0f;  // Scale factor (counts/lb) - typical for load cells, adjust with cal command
  int32_t zeroOffset = 0;          // Tare offset (raw ADC value at zero load)
};

// Initialize load cell interface
bool begin(const Config& cfg = Config{});

// Read raw ADC value from HX711 (blocking, ~10ms)
// Returns true if read successful, false if timeout
bool readRaw(int32_t& value);

// Read weight in pounds (applies calibration and zero offset)
// Returns true if read successful
bool read(float& weightLbs);

// Tare the scale (set current reading as zero)
// Takes multiple samples and averages for stability
bool tare(uint8_t samples = 10);

// Calibrate with a known weight (must tare first with no weight)
// knownWeightLbs: weight of calibration mass in pounds
// Returns true if calibration successful
bool calibrate(float knownWeightLbs, uint8_t samples = 10);

// Set calibration factor (kg per raw ADC unit)
// To calibrate: place known weight, read raw value, calculate factor
void setCalibration(float factor);

// Get current calibration factor
float getCalibration();

// Get current zero offset
int32_t getZeroOffset();

// Check if HX711 is ready (data pin low)
bool isReady();

}  // namespace loadcell
