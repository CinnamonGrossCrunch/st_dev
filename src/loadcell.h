#pragma once
// loadcell.h - Load cell interface (supports HX711 and NAU7802 backends)
// Reads weight/force from load cell via selected ADC backend
//
// Backend selection via build flags in platformio.ini:
//   -D LOADCELL_BACKEND_NAU7802   -> Use NAU7802 (I2C)
//   (default)                      -> Use HX711 (GPIO bit-bang)

#include <Arduino.h>

namespace loadcell {

// Configuration
// Note: sckPin/dtPin are only used by HX711 backend; NAU7802 uses I2C
struct Config {
  uint8_t sckPin = 11;   // Clock pin (D11) - HX711 only
  uint8_t dtPin = 12;    // Data pin (D12) - HX711 only
  float calibrationFactor = 800.0f;  // Scale factor (counts/lb) - typical for load cells, adjust with cal command
  int32_t zeroOffset = 0;          // Tare offset (raw ADC value at zero load)
};

// Initialize load cell interface
bool begin(const Config& cfg = Config{});

// Read raw ADC value from load cell (blocking, ~10ms)
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

// Check if ADC is ready (data available)
bool isReady();

// Get backend name for logging
const char* getBackendName();

}  // namespace loadcell
