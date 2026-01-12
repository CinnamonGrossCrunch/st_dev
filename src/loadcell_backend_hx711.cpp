// loadcell_backend_hx711.cpp - HX711 load cell backend implementation
// Original HX711 implementation preserved as a backend option

#include "loadcell_backend.h"

// Only compile this file when HX711 backend is selected (or no backend specified)
#if !defined(LOADCELL_BACKEND_NAU7802)

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

namespace loadcell_backend {

// ---- State variables ----
static BackendConfig s_config;
static bool s_initialized = false;

// Moving average filter for raw readings
static constexpr uint8_t FILTER_SIZE = 10;
static int32_t s_filterBuffer[FILTER_SIZE] = {0};
static uint8_t s_filterIndex = 0;
static bool s_filterFilled = false;
static int32_t s_filteredValue = 0;

// ---- HX711 Protocol ----
// The HX711 uses a simple serial protocol:
// - Data is clocked out MSB first on falling edge of SCK
// - 24 bits of data + 1-3 gain selection pulses
// - Gain: 25 pulses = 128x (Channel A), 26 = 32x (Channel B), 27 = 64x (Channel A)

bool init(const BackendConfig& cfg) {
  s_config = cfg;
  
  pinMode(s_config.sckPin, OUTPUT);
  pinMode(s_config.dtPin, INPUT);
  
  digitalWrite(s_config.sckPin, LOW);
  delay(10);  // Let HX711 settle
  
  s_initialized = true;
  
  // Wait for HX711 to be ready
  uint32_t timeout = millis() + 1000;
  while (!isReady() && millis() < timeout) {
    delay(10);
  }
  
  // Load calibration from storage
  loadCalibration();
  
  return isReady();
}

bool isReady() {
  if (!s_initialized) return false;
  return digitalRead(s_config.dtPin) == LOW;
}

bool readRaw(int32_t& value) {
  if (!s_initialized) {
    return false;
  }
  
  // Wait for HX711 to be ready (max 100ms)
  uint32_t timeout = millis() + 100;
  while (!isReady()) {
    if (millis() > timeout) {
      return false;  // Timeout
    }
    delayMicroseconds(10);
  }
  
  // Read 24 bits (MSB first)
  uint32_t data = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(s_config.sckPin, HIGH);
    delayMicroseconds(1);
    data = (data << 1) | digitalRead(s_config.dtPin);
    digitalWrite(s_config.sckPin, LOW);
    delayMicroseconds(1);
  }
  
  // Set gain for next reading (25 pulses = 128x gain on Channel A)
  digitalWrite(s_config.sckPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(s_config.sckPin, LOW);
  delayMicroseconds(1);
  
  // Convert to signed 24-bit value
  if (data & 0x800000) {
    data |= 0xFF000000;  // Sign extend
  }
  
  value = (int32_t)data;
  
  // Invert sign - load cell is wired for compression but we're using tension
  value = -value;
  
  // Sanity check: reject if value is suspiciously at limits
  // This catches HX711 errors/resets
  if (value == 0 || value == -1 || value == 0x7FFFFF || value == -0x800000) {
    return false;  // Bad reading
  }
  
  return true;
}

bool read(float& weightLbs) {
  int32_t raw;
  if (!readRaw(raw)) {
    return false;
  }
  
  // Reject obviously bad readings (HX711 can glitch)
  // Typical range for 200kg load cell is ±8,388,607 (24-bit ADC)
  // Reject if reading is suspiciously close to 0 or max
  if (abs(raw) < 100 || abs(raw) > 8000000) {
    return false;  // Skip this bad reading
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
  if (s_config.calibrationFactor <= 0.0f) {
    // Not calibrated - return raw value scaled arbitrarily so user sees something
    weightLbs = (float)(s_filteredValue - s_config.zeroOffset) / 10000.0f;
    return true;  // Still return true so teleplot shows data
  }
  weightLbs = (float)(s_filteredValue - s_config.zeroOffset) / s_config.calibrationFactor;
  return true;
}

bool tare(uint8_t samples) {
  if (!s_initialized) {
    return false;
  }
  
  int64_t sum = 0;
  uint8_t validSamples = 0;
  
  for (uint8_t i = 0; i < samples; i++) {
    int32_t raw;
    if (readRaw(raw)) {
      sum += raw;
      validSamples++;
    }
    delay(10);  // Small delay between samples
  }
  
  if (validSamples == 0) {
    return false;
  }
  
  s_config.zeroOffset = (int32_t)(sum / validSamples);
  
  // Reset moving average filter to avoid transients
  s_filterIndex = 0;
  s_filterFilled = false;
  for (uint8_t i = 0; i < FILTER_SIZE; i++) {
    s_filterBuffer[i] = s_config.zeroOffset;
  }
  
  // Save to persistent storage
  saveCalibration();
  
  return true;
}

bool calibrate(float knownWeightLbs, uint8_t samples) {
  if (!s_initialized || knownWeightLbs <= 0.0f) {
    return false;
  }
  
  int64_t sum = 0;
  uint8_t validSamples = 0;
  
  for (uint8_t i = 0; i < samples; i++) {
    int32_t raw;
    if (readRaw(raw)) {
      sum += raw;
      validSamples++;
    }
    delay(10);
  }
  
  if (validSamples == 0) {
    return false;
  }
  
  int32_t avgRaw = (int32_t)(sum / validSamples);
  int32_t delta = avgRaw - s_config.zeroOffset;
  
  if (abs(delta) < 100) {
    return false;  // Delta too small - weight not detected or bad tare
  }
  
  // Calibration factor = delta_counts / known_weight_lbs
  s_config.calibrationFactor = (float)delta / knownWeightLbs;
  
  // Save to persistent storage
  saveCalibration();
  
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

static constexpr uint32_t CAL_VERSION = 1;
static const char* CAL_FILENAME = "/loadcell_cal.bin";

struct CalibrationData {
  uint32_t version;
  int32_t zeroOffset;
  float calibrationFactor;
  uint32_t checksum;
};

// Filesystem initialization flag
static bool s_fsInitialized = false;

static bool ensureFS() {
  if (s_fsInitialized) return true;
  s_fsInitialized = InternalFS.begin();
  return s_fsInitialized;
}

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
    Serial.println("loadcell:HX711:FS mount failed");
    return false;
  }
  
  CalibrationData data;
  data.version = CAL_VERSION;
  data.zeroOffset = s_config.zeroOffset;
  data.calibrationFactor = s_config.calibrationFactor;
  data.checksum = computeChecksum(data);
  
  File file(InternalFS);
  if (!file.open(CAL_FILENAME, FILE_O_WRITE)) {
    Serial.println("loadcell:HX711:save open failed");
    return false;
  }
  
  size_t written = file.write((uint8_t*)&data, sizeof(data));
  file.close();
  
  if (written != sizeof(data)) {
    Serial.println("loadcell:HX711:save write failed");
    return false;
  }
  
  Serial.println("loadcell:HX711:calibration saved");
  return true;
}

bool loadCalibration() {
  if (!ensureFS()) {
    Serial.println("loadcell:HX711:FS mount failed");
    return false;
  }
  
  File file(InternalFS);
  if (!file.open(CAL_FILENAME, FILE_O_READ)) {
    Serial.println("loadcell:HX711:no saved calibration");
    return false;
  }
  
  CalibrationData data;
  size_t bytesRead = file.read((uint8_t*)&data, sizeof(data));
  file.close();
  
  if (bytesRead != sizeof(data)) {
    Serial.println("loadcell:HX711:cal file corrupt (size)");
    return false;
  }
  
  if (data.version != CAL_VERSION) {
    Serial.println("loadcell:HX711:cal file version mismatch");
    return false;
  }
  
  uint32_t checksum = computeChecksum(data);
  if (checksum != data.checksum) {
    Serial.println("loadcell:HX711:cal file corrupt (checksum)");
    return false;
  }
  
  s_config.zeroOffset = data.zeroOffset;
  s_config.calibrationFactor = data.calibrationFactor;
  
  Serial.print("loadcell:HX711:loaded cal factor=");
  Serial.print(s_config.calibrationFactor);
  Serial.print(" offset=");
  Serial.println(s_config.zeroOffset);
  
  return true;
}

}  // namespace loadcell_backend

#endif  // !LOADCELL_BACKEND_NAU7802
