#pragma once
// bleservice.h - Bluetooth Low Energy module for Feather nRF52840 Sense
// Uses Adafruit Bluefruit52Lib (bundled with framework)
// NOTE: Named 'bleservice' to avoid conflict with Nordic SDK's 'ble.h'

#include <Arduino.h>

namespace ble {

// Configuration
struct Config {
  const char* deviceName = "StrongTrak";
  int8_t txPower = 0;  // dBm: -40, -20, -16, -12, -8, -4, 0, +4
};

// Initialize BLE with advertising + UART service
// Returns true on success
bool init(const Config& cfg = Config{});

// Returns true if a central is connected
bool isConnected();

// Returns true if BLE is initialized and advertising
bool isAdvertising();

// Send a string over BLE UART (Nordic UART Service)
// Safe to call even if not connected (will be ignored)
void send(const char* str);
void send(const String& str);

// Printf-style send (max 128 chars)
void sendf(const char* fmt, ...);

// Check for incoming data from BLE UART RX
int available();

// Read one byte from BLE UART RX
char read();

// Call periodically in loop() if you want to handle BLE events
// (optional - Bluefruit handles most things via callbacks)
void update();

}  // namespace ble
