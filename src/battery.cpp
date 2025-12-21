// battery.cpp - Battery voltage monitoring implementation
// The Feather nRF52840 has a voltage divider (2x 100K resistors) connected
// to analog pin VBAT_PIN. This divides battery voltage by 2, allowing
// measurement of 0-6V range with the 3.3V ADC reference.
//
// Voltage calculation:
//   ADC is 12-bit (0-4095), reference is 3.6V (nRF52840 internal)
//   Voltage divider ratio is 2:1
//   Battery voltage = (ADC / 4095) * 3.6V * 2
//
// USB Detection:
//   When USB is connected, the voltage reads ~5V (USB power rail).
//   When on battery only, voltage is 3.0-4.2V (LiPo range).
//   We use 4.5V as the threshold to distinguish USB from battery.

#include "battery.h"

// On nRF52840 Feather, VBAT is connected to P0.29 (A6) via voltage divider
// The Adafruit BSP defines PIN_VBAT as 32 (which maps to A6 / P0.29)
#ifndef PIN_VBAT
  #define PIN_VBAT A6
#endif

namespace battery {

// File-scope state
static Config s_config;
static float s_smoothedVoltage = 0.0f;
static bool s_initialized = false;
static char s_statusBuffer[12];  // Buffer for status string

// ADC reference voltage for nRF52840 (internal 3.6V reference)
static constexpr float kAdcRefVoltage = 3.6f;

// ADC resolution (12-bit = 4096 levels)
static constexpr float kAdcMaxValue = 4095.0f;

// Voltage divider ratio (2x 100K resistors = divide by 2)
static constexpr float kVoltageDividerRatio = 2.0f;

void begin(const Config& cfg) {
  s_config = cfg;
  
  // Configure ADC for battery reading
  // Use 12-bit resolution and internal 3.6V reference
  analogReference(AR_INTERNAL_3_0);  // 3.0V reference (closest to 3.3V)
  analogReadResolution(12);          // 12-bit (0-4095)
  
  // Note: AR_INTERNAL_3_0 is actually 3.6V on nRF52840 despite the name
  // This is a quirk of the Adafruit BSP
  
  // Take initial reading to prime the smoothed value
  pinMode(PIN_VBAT, INPUT);
  delay(1);  // Let ADC settle
  
  int raw = analogRead(PIN_VBAT);
  s_smoothedVoltage = (raw / kAdcMaxValue) * kAdcRefVoltage * kVoltageDividerRatio;
  
  s_initialized = true;
}

int readRaw() {
  if (!s_initialized) return 0;
  return analogRead(PIN_VBAT);
}

float readVoltage() {
  if (!s_initialized) return 0.0f;
  
  int raw = analogRead(PIN_VBAT);
  float voltage = (raw / kAdcMaxValue) * kAdcRefVoltage * kVoltageDividerRatio;
  
  // Apply EMA smoothing to reduce noise
  s_smoothedVoltage = s_config.alpha * voltage + (1.0f - s_config.alpha) * s_smoothedVoltage;
  
  return s_smoothedVoltage;
}

bool isUsbPowered() {
  // If voltage is above the USB threshold, we're on USB power
  return readVoltage() > s_config.voltageUsb;
}

PowerSource getPowerSource() {
  if (!s_initialized) return PowerSource::UNKNOWN;
  
  float v = readVoltage();
  if (v > s_config.voltageUsb) {
    return PowerSource::USB;
  }
  return PowerSource::BATTERY;
}

int readPercent() {
  // If on USB power, return -1 to indicate battery reading isn't meaningful
  if (isUsbPowered()) {
    return -1;
  }
  
  float voltage = s_smoothedVoltage;  // Use already-read value
  
  // Clamp to valid range
  if (voltage <= s_config.voltageEmpty) return 0;
  if (voltage >= s_config.voltageFull) return 100;
  
  // Linear interpolation between empty and full
  // Note: LiPo discharge curves are NOT linear, but this is a reasonable approximation.
  // For more accuracy, use a lookup table based on your specific battery.
  float range = s_config.voltageFull - s_config.voltageEmpty;
  float normalized = (voltage - s_config.voltageEmpty) / range;
  
  return (int)(normalized * 100.0f);
}

bool isLow() {
  if (isUsbPowered()) return false;  // Not low if on USB
  int pct = readPercent();
  return pct >= 0 && pct < 10;
}

bool isCritical() {
  if (isUsbPowered()) return false;  // Not critical if on USB
  int pct = readPercent();
  return pct >= 0 && pct < 5;
}

const char* getStatusString() {
  if (isUsbPowered()) {
    return "USB";
  }
  
  int pct = readPercent();
  
  if (pct < 0) {
    return "???";
  } else if (pct < 5) {
    return "CRITICAL";
  } else if (pct < 10) {
    return "LOW";
  } else {
    // Format as "XX%"
    snprintf(s_statusBuffer, sizeof(s_statusBuffer), "%d%%", pct);
    return s_statusBuffer;
  }
}

}  // namespace battery
