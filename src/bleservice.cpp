// bleservice.cpp - Bluetooth Low Energy module implementation
#include "bleservice.h"

#include <bluefruit.h>

namespace ble {

// Nordic UART Service (NUS) for streaming text
static BLEUart bleuart;

// Connection state
static volatile bool connected = false;
static bool s_initSuccess = false;

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
  Serial.println("ble:init starting...");
  
  // Initialize Bluefruit
  if (!Bluefruit.begin()) {
    Serial.println("ble:init_failed - Bluefruit.begin() returned false");
    return false;
  }
  Serial.println("ble:Bluefruit.begin() OK");

  Bluefruit.setTxPower(cfg.txPower);
  Bluefruit.setName(cfg.deviceName);
  Serial.print("ble:name set to ");
  Serial.println(cfg.deviceName);

  // Set connection callbacks
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // Add UART service
  bleuart.begin();
  Serial.println("ble:UART service started");

  // Set up advertising - keep it minimal to avoid packet overflow
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(bleuart);
  
  // Put name in scan response (more room there)
  Bluefruit.ScanResponse.addName();

  // Advertising parameters
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // Standard intervals (in 0.625ms units)
  Bluefruit.Advertising.setFastTimeout(30);    // seconds in fast mode
  
  // Start advertising
  if (!Bluefruit.Advertising.start(0)) {        // 0 = advertise forever
    Serial.println("ble:Advertising.start() FAILED");
    return false;
  }

  Serial.print("ble:advertising started as '");
  Serial.print(cfg.deviceName);
  Serial.println("'");
  s_initSuccess = true;
  return true;
}

bool isConnected() {
  return connected;
}

bool isAdvertising() {
  return s_initSuccess && !connected;
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

int available() {
  return bleuart.available();
}

char read() {
  return bleuart.read();
}

void update() {
  // Bluefruit uses SoftDevice callbacks; nothing required here for basic UART.
  // Use available() and read() to process incoming commands.
}

}  // namespace ble
