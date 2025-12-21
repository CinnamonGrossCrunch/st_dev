#pragma once
// battery.h - Battery voltage monitoring for Adafruit Feather nRF52840
// Reads battery voltage via the onboard voltage divider on pin A6 (VBAT).
// Works with LiPo batteries (3.0V empty to 4.2V full).
// Detects USB power and reports accordingly.

#include <Arduino.h>

namespace battery {

// Power source enumeration
enum class PowerSource {
  BATTERY,    // Running on LiPo battery
  USB,        // USB power connected (voltage reads ~5V)
  UNKNOWN     // Unable to determine
};

// Battery voltage thresholds for LiPo (single cell)
// These are approximate and vary by battery chemistry and load.
struct Config {
  float voltageEmpty = 3.2f;   // Voltage at 0% (don't go below 3.0V!)
  float voltageFull = 4.2f;    // Voltage at 100% (fully charged LiPo)
  float voltageUsb = 4.5f;     // Above this, assume USB power (not battery)
  float alpha = 0.1f;          // EMA smoothing (lower = more smoothing)
};

// Initialize battery monitoring. Call once in setup().
void begin(const Config& cfg = Config{});

// Read current battery voltage (volts). Returns smoothed value.
// Note: When on USB power, this returns ~5V (USB voltage, not battery).
float readVoltage();

// Get battery percentage (0-100). Uses configured voltage thresholds.
// Returns -1 if on USB power (battery reading not meaningful).
int readPercent();

// Get raw ADC value (for debugging)
int readRaw();

// Check if USB power is connected (voltage > 4.5V threshold)
bool isUsbPowered();

// Get current power source
PowerSource getPowerSource();

// Check if battery is low (below 10%) - only valid when on battery
bool isLow();

// Check if battery is critical (below 5%) - only valid when on battery
bool isCritical();

// Get a human-readable status string
// Returns "USB", "100%", "75%", "LOW", "CRITICAL", etc.
const char* getStatusString();

}  // namespace battery
