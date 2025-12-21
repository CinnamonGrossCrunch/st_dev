// main.cpp - Feather Sense IMU Test (modular version)
// Reads IMU + optional magnetometer and streams to Teleplot via Serial.
// Accepts serial commands: "tare" to reset position origin.

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "imu.h"
#include "teleplot.h"
#include "bleservice.h"
#include "position.h"
#include "battery.h"

// ---- Timing configuration ----
static constexpr uint32_t kSampleIntervalMs = 20;   // ~50 Hz
static constexpr uint32_t kHeartbeatIntervalMs = 1000;
static constexpr uint32_t kStatusIntervalMs = 3000;
static constexpr uint32_t kBleUpdateIntervalMs = 100;  // 10 Hz for BLE (conserve bandwidth)
static constexpr uint32_t kBatteryIntervalMs = 5000;   // Battery check every 5 seconds (slow-changing)

// ---- Timing state ----
static uint32_t s_lastSampleMs = 0;
static uint32_t s_lastHeartbeatMs = 0;
static uint32_t s_lastStatusMs = 0;
static uint32_t s_lastBleUpdateMs = 0;
static uint32_t s_lastBatteryMs = 0;

// ---- Serial command buffer ----
static char s_cmdBuffer[32];
static uint8_t s_cmdIndex = 0;

// Process incoming serial commands (non-blocking)
void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    
    // Handle end of command (newline or carriage return)
    if (c == '\n' || c == '\r') {
      if (s_cmdIndex > 0) {
        s_cmdBuffer[s_cmdIndex] = '\0';  // Null-terminate
        
        // Parse command (case-insensitive)
        if (strcasecmp(s_cmdBuffer, "tare") == 0) {
          position::tare();
          teleplot::log("CMD: position tared");
        } 
        else if (strcasecmp(s_cmdBuffer, "help") == 0) {
          teleplot::log("Commands: tare, help, status");
        }
        else if (strcasecmp(s_cmdBuffer, "status") == 0) {
          teleplot::logKV("imu_ok", imu::imuOk() ? 1 : 0);
          teleplot::logKV("mag_present", imu::magOk() ? 1 : 0);
          teleplot::logKV("ble_connected", ble::isConnected() ? 1 : 0);
          teleplot::logKV("power_source", battery::isUsbPowered() ? "USB" : "BATTERY");
          teleplot::logKV("battery_status", battery::getStatusString());
        }
        else if (s_cmdIndex > 0) {
          teleplot::log("CMD: unknown (try 'help')");
        }
        
        s_cmdIndex = 0;  // Reset buffer
      }
    } 
    else if (s_cmdIndex < sizeof(s_cmdBuffer) - 1) {
      // Add character to buffer (if room)
      s_cmdBuffer[s_cmdIndex++] = c;
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(50);

  Wire.begin();
  teleplot::begin();

  teleplot::log("boot: imu_test");

  if (!imu::begin()) {
    teleplot::log("ERROR: IMU init failed");
  } else {
    teleplot::logKV("imu_model", imu::imuModelName());
  }
  teleplot::logKV("imu_ok", imu::imuOk() ? 1 : 0);
  teleplot::logKV("mag_present", imu::magOk() ? 1 : 0);

  // Initialize BLE
  ble::Config bleCfg;
  bleCfg.deviceName = "StrongTrak";
  if (!ble::init(bleCfg)) {
    teleplot::log("WARN: BLE init failed");
  }

  // Initialize position tracking
  position::begin();
  delay(500);  // Let IMU settle
  position::tare();  // Set current position as origin
  teleplot::log("position:tared");

  // Initialize battery monitoring
  battery::begin();
  teleplot::logKV("power_source", battery::isUsbPowered() ? "USB" : "BATTERY");
  teleplot::logKV("battery_status", battery::getStatusString());

  teleplot::log("stream:ready");
  teleplot::log("Type 'help' for commands");
}

void loop() {
  const uint32_t nowMs = millis();

  // Process any incoming serial commands (non-blocking)
  processSerialCommands();

  // LED heartbeat at 2 Hz.
  digitalWrite(LED_BUILTIN, (nowMs / 250) % 2);

  // Console heartbeat at 1 Hz.
  if ((nowMs - s_lastHeartbeatMs) >= kHeartbeatIntervalMs) {
    s_lastHeartbeatMs = nowMs;
    teleplot::logKV("heartbeat_ms", (int)nowMs);
  }

  // Periodic status (useful if serial opened after boot).
  if ((nowMs - s_lastStatusMs) >= kStatusIntervalMs) {
    s_lastStatusMs = nowMs;
    teleplot::logKV("imu_ok", imu::imuOk() ? 1 : 0);
    teleplot::logKV("mag_present", imu::magOk() ? 1 : 0);
  }

  // Battery monitoring (slow update - battery changes slowly)
  if ((nowMs - s_lastBatteryMs) >= kBatteryIntervalMs) {
    s_lastBatteryMs = nowMs;
    float battV = battery::readVoltage();
    
    // Log voltage always (useful for debugging)
    teleplot::emit("battery_V", battV);
    
    if (battery::isUsbPowered()) {
      // On USB power - show status but don't emit percentage plot
      teleplot::logKV("power_source", "USB");
    } else {
      // On battery - emit percentage and check for low battery
      int battPct = battery::readPercent();
      teleplot::emit("battery_pct", battPct);
      teleplot::logKV("battery_status", battery::getStatusString());
      
      // Warn if battery is low
      if (battery::isCritical()) {
        teleplot::log("WARN: Battery CRITICAL (<5%)");
      } else if (battery::isLow()) {
        teleplot::log("WARN: Battery low (<10%)");
      }
    }
  }

  // If IMU not available, only heartbeat/status.
  if (!imu::imuOk()) return;

  // Sample at configured rate.
  if ((nowMs - s_lastSampleMs) < kSampleIntervalMs) return;
  s_lastSampleMs = nowMs;

  imu::Reading r;
  if (!imu::read(r)) return;

  // Update position tracking
  position::update(r.acc_x, r.acc_y, r.acc_z);
  auto pos = position::getPosition();
  auto vel = position::getVelocity();

  // Emit only Z height (vertical position) - in centimeters for readability
  teleplot::emit("height_cm", pos.z * 100.0f);
  teleplot::emit("vel_z_ms", vel.z);
  
  // Emit raw IMU data (can be disabled for cleaner output)
  // teleplot::emitVec3("acc_g", r.acc_x, r.acc_y, r.acc_z);
  // teleplot::emitVec3("gyro_dps", r.gyro_x, r.gyro_y, r.gyro_z);
  teleplot::emit("imu_temp_C", r.temp);
  
  // Magnetometer (optional)
  if (imu::magOk()) {
    teleplot::emitVec3("mag_uT", r.mag_x, r.mag_y, r.mag_z);
  }

  // Send data over BLE at lower rate (to conserve bandwidth)
  if (ble::isConnected() && (nowMs - s_lastBleUpdateMs) >= kBleUpdateIntervalMs) {
    s_lastBleUpdateMs = nowMs;
    // CSV format for Bluefruit Connect plotter: "height_cm,velocity_m/s\n"
    ble::sendf("%.1f,%.2f\n", pos.z * 100.0f, vel.z);
  }
}
