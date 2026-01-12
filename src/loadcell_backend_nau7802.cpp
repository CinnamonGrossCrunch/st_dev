// loadcell_backend_nau7802.cpp - NAU7802 load cell backend implementation
// Uses Adafruit NAU7802 library over I2C

#include "loadcell_backend.h"

// Only compile this file when NAU7802 backend is selected
#if defined(LOADCELL_BACKEND_NAU7802)

#include <Wire.h>
#include <Adafruit_NAU7802.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <cmath>  // for fabs()

using namespace Adafruit_LittleFS_Namespace;

namespace loadcell_backend {

// ---- State variables ----
static BackendConfig s_config;
static bool s_initialized = false;
static Adafruit_NAU7802 nau;

// Moving average filter for raw readings
static constexpr uint8_t FILTER_SIZE = 10;
static int32_t s_filterBuffer[FILTER_SIZE] = {0};
static uint8_t s_filterIndex = 0;
static bool s_filterFilled = false;
static int32_t s_filteredValue = 0;

// Debug counter
static uint8_t s_readDebugCount = 0;

bool init(const BackendConfig& cfg) {
  s_config = cfg;
  
  // NAU7802 uses I2C - Wire should already be initialized by IMU
  // but we ensure it's ready
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C
  
  // Initialize NAU7802
  if (!nau.begin()) {
    Serial.println("loadcell:NAU7802:init failed - not found on I2C");
    return false;
  }
  
  // Configure NAU7802
  // Gain: 128x for load cells (maximum sensitivity)
  if (!nau.setGain(NAU7802_GAIN_128)) {
    Serial.println("loadcell:NAU7802:setGain failed");
    return false;
  }
  
  // Sample rate: 80 SPS (good balance of speed and noise)
  if (!nau.setRate(NAU7802_RATE_80SPS)) {
    Serial.println("loadcell:NAU7802:setRate failed");
    return false;
  }
  
  // Enable the ADC
  if (!nau.enable(true)) {
    Serial.println("loadcell:NAU7802:enable failed");
    return false;
  }
  
  // NOTE: Skipping internal calibrations - they may cause drift
  // The NAU7802 internal cal can interfere with software calibration
  // We'll rely purely on software zero offset and calibration factor
  
  s_initialized = true;
  
  // Load saved calibration from flash
  loadCalibration();
  
  Serial.println("loadcell:NAU7802:initialized");
  Serial.print("loadcell:NAU7802:cal_factor=");
  Serial.print(s_config.calibrationFactor);
  Serial.print(" zero_offset=");
  Serial.println(s_config.zeroOffset);
  
  return true;
}

bool isReady() {
  if (!s_initialized) return false;
  return nau.available();
}

bool readRaw(int32_t& value) {
  if (!s_initialized) {
    return false;
  }
  
  // Wait for data ready (max 100ms = ~8 samples at 80 SPS)
  uint32_t timeout = millis() + 100;
  while (!nau.available()) {
    if (millis() > timeout) {
      return false;  // Timeout
    }
    delayMicroseconds(100);
  }
  
  // Read the 24-bit signed value
  value = nau.read();
  
  // Note: NAU7802 returns signed 24-bit value, already properly formatted
  // No sign extension needed like HX711
  
  // Sanity check: reject if value is at limits (indicates error)
  if (value == 0x7FFFFF || value == -0x800000) {
    return false;  // Saturated - bad reading
  }
  
  return true;
}

bool read(float& weightLbs) {
  int32_t raw;
  if (!readRaw(raw)) {
    return false;
  }
  
  // Debug: log raw values for first 5 reads after calibration
  bool shouldDebug = (s_readDebugCount < 5);
  if (shouldDebug) {
    Serial.print("loadcell:NAU7802:read raw=");
    Serial.print(raw);
    Serial.print(" filteredValue=");
    Serial.println(s_filteredValue);
  }
  
  // Reject obviously bad readings
  // NAU7802 24-bit ADC range: ±8,388,607
  if (abs(raw) > 8000000) {
    return false;  // Saturated or error
  }
  
  // Add to moving average filter
  s_filterBuffer[s_filterIndex] = raw;
  s_filterIndex = (s_filterIndex + 1) % FILTER_SIZE;
  
  if (!s_filterFilled && s_filterIndex == 0) {
    s_filterFilled = true;
  }
  
  // Calculate average (only if filter is full)
  if (s_filterFilled) {
    int64_t sum = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
      sum += s_filterBuffer[i];
    }
    s_filteredValue = (int32_t)(sum / FILTER_SIZE);
  } else {
    s_filteredValue = raw;  // Use raw until filter fills
  }
  
  // Apply zero offset and calibration
  // calibrationFactor is in counts/lb, so divide to get lbs
  // NOTE: NAU7802 can produce negative counts (opposite polarity from HX711),
  // so calibrationFactor may be negative. Check for uncalibrated using near-zero.
  if (fabs(s_config.calibrationFactor) < 1.0f) {
    // Not calibrated - return raw value scaled arbitrarily so user sees something
    weightLbs = (float)(s_filteredValue - s_config.zeroOffset) / 10000.0f;
    return true;  // Still return true so teleplot shows data
  }
  weightLbs = (float)(s_filteredValue - s_config.zeroOffset) / s_config.calibrationFactor;
  
  // Debug: log calculation for first few reads after calibration
  if (shouldDebug) {
    Serial.print("loadcell:NAU7802:calc weight=(");
    Serial.print(s_filteredValue);
    Serial.print(" - ");
    Serial.print(s_config.zeroOffset);
    Serial.print(") / ");
    Serial.print(s_config.calibrationFactor);
    Serial.print(" = ");
    Serial.println(weightLbs);
    s_readDebugCount++;  // Increment after all debug output
  }
  
  return true;
}

bool tare(uint8_t samples) {
  if (!s_initialized) {
    return false;
  }
  
  // Use more samples for NAU7802 (it's faster and cleaner)
  if (samples < 20) samples = 20;
  
  int64_t sum = 0;
  uint8_t validSamples = 0;
  
  Serial.print("loadcell:NAU7802:taring with ");
  Serial.print(samples);
  Serial.println(" samples...");
  
  for (uint8_t i = 0; i < samples; i++) {
    int32_t raw;
    if (readRaw(raw)) {
      sum += raw;
      validSamples++;
    }
    // At 80 SPS, each sample takes ~12.5ms
    // Wait longer to ensure we get a fresh sample, not cached data
    delay(15);
  }
  
  if (validSamples == 0) {
    Serial.println("loadcell:NAU7802:tare failed - no valid samples");
    return false;
  }
  
  s_config.zeroOffset = (int32_t)(sum / validSamples);
  
  // Reset moving average filter to avoid transients
  s_filterIndex = 0;
  s_filterFilled = false;
  for (uint8_t i = 0; i < FILTER_SIZE; i++) {
    s_filterBuffer[i] = s_config.zeroOffset;
  }
  
  Serial.print("loadcell:NAU7802:tared, offset=");
  Serial.println(s_config.zeroOffset);
  
  // Save to persistent storage
  saveCalibration();
  
  return true;
}

bool calibrate(float knownWeightLbs, uint8_t samples) {
  if (!s_initialized || knownWeightLbs <= 0.0f) {
    Serial.println("loadcell:NAU7802:calibrate failed - invalid weight");
    return false;
  }
  
  // Use more samples for calibration
  if (samples < 20) samples = 20;
  
  int64_t sum = 0;
  uint8_t validSamples = 0;
  
  Serial.print("loadcell:NAU7802:calibrating with ");
  Serial.print(knownWeightLbs);
  Serial.println(" lbs...");
  
  for (uint8_t i = 0; i < samples; i++) {
    int32_t raw;
    if (readRaw(raw)) {
      sum += raw;
      validSamples++;
    }
    // At 80 SPS, need at least 12.5ms between samples
    delay(15);
  }
  
  if (validSamples == 0) {
    Serial.println("loadcell:NAU7802:calibrate failed - no valid samples");
    return false;
  }
  
  int32_t avgRaw = (int32_t)(sum / validSamples);
  int32_t delta = avgRaw - s_config.zeroOffset;
  
  Serial.print("loadcell:NAU7802:avgRaw=");
  Serial.print(avgRaw);
  Serial.print(" zeroOffset=");
  Serial.print(s_config.zeroOffset);
  Serial.print(" delta=");
  Serial.println(delta);
  
  // NAU7802 is more precise, but still need minimum delta
  if (abs(delta) < 50) {
    Serial.println("loadcell:NAU7802:calibrate failed - delta too small");
    return false;
  }
  
  // Calibration factor = delta_counts / known_weight_lbs
  s_config.calibrationFactor = (float)delta / knownWeightLbs;
  
  Serial.print("loadcell:NAU7802:calibrated, factor=");
  Serial.print(s_config.calibrationFactor);
  Serial.print(" (delta=");
  Serial.print(delta);
  Serial.print(" / weight=");
  Serial.print(knownWeightLbs);
  Serial.println(")");
  
  // Reset moving average filter to avoid transients
  // IMPORTANT: Keep s_filterIndex at 0 so first read() overwrites position 0
  // This maintains the pre-filled average instead of diluting it
  s_filterIndex = 0;
  s_filterFilled = true;  // Mark as filled immediately
  for (uint8_t i = 0; i < FILTER_SIZE; i++) {
    s_filterBuffer[i] = avgRaw;  // Pre-fill with calibration reading
  }
  s_filteredValue = avgRaw;
  
  Serial.print("loadcell:NAU7802:filter pre-filled, s_filteredValue=");
  Serial.println(s_filteredValue);
  
  // Reset debug counter to log next 5 reads
  s_readDebugCount = 0;
  
  // Save to persistent storage
  saveCalibration();
  
  // Verify saved values (without reloading - keep RAM values intact)
  Serial.print("loadcell:NAU7802:calibration saved - factor=");
  Serial.print(s_config.calibrationFactor);
  Serial.print(" offset=");
  Serial.println(s_config.zeroOffset);
  
  return true;
}

void setCalibration(float factor) {
  s_config.calibrationFactor = factor;
}

float getCalibration() {
  return s_config.calibrationFactor;
}

int32_t getZeroOffset() {
  return s_config.zeroOffset;
}

// ---- Persistence using InternalFileSystem (LittleFS on nRF52) ----
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

// Filesystem initialization flag
static bool s_fsInitialized = false;

static bool ensureFS() {
  if (s_fsInitialized) return true;
  s_fsInitialized = InternalFS.begin();
  return s_fsInitialized;
}

static constexpr uint32_t CAL_VERSION = 2;  // Different version from HX711
static const char* CAL_FILENAME = "/loadcell_nau_cal.bin";

struct CalibrationData {
  uint32_t version;
  int32_t zeroOffset;
  float calibrationFactor;
  uint32_t checksum;
};

static uint32_t computeChecksum(const CalibrationData& data) {
  // Simple checksum: XOR all bytes except checksum field
  const uint8_t* bytes = (const uint8_t*)&data;
  uint32_t sum = 0;
  for (size_t i = 0; i < offsetof(CalibrationData, checksum); i++) {
    sum ^= bytes[i] << ((i % 4) * 8);
  }
  return sum;
}

bool saveCalibration() {
  if (!ensureFS()) {
    Serial.println("loadcell:NAU7802:FS mount failed");
    return false;
  }
  
  CalibrationData data;
  data.version = CAL_VERSION;
  data.zeroOffset = s_config.zeroOffset;
  data.calibrationFactor = s_config.calibrationFactor;
  data.checksum = computeChecksum(data);
  
  Serial.print("loadcell:NAU7802:saving struct - version=");
  Serial.print(data.version);
  Serial.print(" offset=");
  Serial.print(data.zeroOffset);
  Serial.print(" factor=");
  Serial.print(data.calibrationFactor, 2);
  Serial.print(" checksum=");
  Serial.println(data.checksum);
  
  // Delete old file first to ensure clean write
  if (InternalFS.exists(CAL_FILENAME)) {
    InternalFS.remove(CAL_FILENAME);
  }
  
  File file(InternalFS);
  if (!file.open(CAL_FILENAME, FILE_O_WRITE)) {
    Serial.println("loadcell:NAU7802:save open failed");
    return false;
  }
  
  size_t written = file.write((uint8_t*)&data, sizeof(data));
  file.flush();  // Ensure data is written to flash
  file.close();
  
  if (written != sizeof(data)) {
    Serial.println("loadcell:NAU7802:save write failed");
    return false;
  }
  
  Serial.println("loadcell:NAU7802:calibration saved");
  return true;
}

bool loadCalibration() {
  if (!ensureFS()) {
    Serial.println("loadcell:NAU7802:FS mount failed");
    return false;
  }
  
  File file(InternalFS);
  if (!file.open(CAL_FILENAME, FILE_O_READ)) {
    Serial.println("loadcell:NAU7802:no saved calibration");
    return false;
  }
  
  CalibrationData data;
  size_t bytesRead = file.read((uint8_t*)&data, sizeof(data));
  file.close();
  
  if (bytesRead != sizeof(data)) {
    Serial.println("loadcell:NAU7802:cal file corrupt (size)");
    return false;
  }
  
  if (data.version != CAL_VERSION) {
    Serial.println("loadcell:NAU7802:cal file version mismatch");
    return false;
  }
  
  uint32_t checksum = computeChecksum(data);
  if (checksum != data.checksum) {
    Serial.println("loadcell:NAU7802:cal file corrupt (checksum)");
    return false;
  }
  
  s_config.zeroOffset = data.zeroOffset;
  s_config.calibrationFactor = data.calibrationFactor;
  
  Serial.print("loadcell:NAU7802:loaded struct - version=");
  Serial.print(data.version);
  Serial.print(" offset=");
  Serial.print(data.zeroOffset);
  Serial.print(" factor=");
  Serial.print(data.calibrationFactor, 2);
  Serial.print(" checksum=");
  Serial.print(data.checksum);
  Serial.print(" (computed=");
  Serial.print(checksum);
  Serial.println(")");
  
  Serial.print("loadcell:NAU7802:loaded cal factor=");
  Serial.print(s_config.calibrationFactor);
  Serial.print(" offset=");
  Serial.println(s_config.zeroOffset);
  
  return true;
}

}  // namespace loadcell_backend

#endif  // LOADCELL_BACKEND_NAU7802
