// bleservice.cpp - Bluetooth Low Energy module implementation
#include "bleservice.h"

#include <bluefruit.h>

namespace ble {

// Nordic UART Service (NUS) for streaming text
static BLEUart bleuart;

// Connection state
static volatile bool connected = false;

// Callbacks
static void connectCallback(uint16_t connHandle) {
  (void)connHandle;
  connected = true;
  Serial.println("ble:connected");
}

static void disconnectCallback(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  (void)reason;
  connected = false;
  Serial.println("ble:disconnected");
}

bool init(const Config& cfg) {
  // Initialize Bluefruit
  if (!Bluefruit.begin()) {
    Serial.println("ble:init_failed");
    return false;
  }

  Bluefruit.setTxPower(cfg.txPower);
  Bluefruit.setName(cfg.deviceName);

  // Set connection callbacks
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // Add UART service
  bleuart.begin();

  // Set up advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  // Advertising parameters
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = advertise forever

  Serial.print("ble:advertising:");
  Serial.println(cfg.deviceName);
  return true;
}

bool isConnected() {
  return connected;
}

void send(const char* str) {
  if (connected && str) {
    bleuart.write(str);
  }
}

void send(const String& str) {
  send(str.c_str());
}

void sendf(const char* fmt, ...) {
  if (!connected) return;
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  bleuart.write(buf);
}

void update() {
  // Bluefruit uses SoftDevice callbacks; nothing required here for basic UART.
  // Could poll bleuart.available() for incoming commands if needed.
}

}  // namespace ble
