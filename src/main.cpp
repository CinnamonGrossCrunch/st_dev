// main.cpp - Feather Sense IMU Test (modular version)
// Reads IMU + optional magnetometer and streams to Teleplot via Serial.
// Accepts serial/BLE commands: "tare", "cal <lbs>", "status", "help"

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "imu.h"
#include "teleplot.h"
#include "bleservice.h"
#include "position.h"
#include "battery.h"
#include "loadcell.h"

// ---- Timing configuration ----
static constexpr uint32_t kSampleIntervalMs = 50;   // 20 Hz (was 50Hz - too fast for stable I2C)
static constexpr uint32_t kHeartbeatIntervalMs = 1000;
static constexpr uint32_t kStatusIntervalMs = 3000;
static constexpr uint32_t kBleUpdateIntervalMs = 100;  // 10 Hz for BLE (conserve bandwidth)
static constexpr uint32_t kBatteryIntervalMs = 5000;   // Battery check every 5 seconds (slow-changing)
static constexpr uint32_t kLoadCellIntervalMs = 200;   // Load cell 5 Hz (was 10Hz - slower for stability)

// ---- Timing state ----
static uint32_t s_lastSampleMs = 0;
static uint32_t s_lastHeartbeatMs = 0;
static uint32_t s_lastStatusMs = 0;
static uint32_t s_lastBleUpdateMs = 0;
static uint32_t s_lastBatteryMs = 0;
static uint32_t s_lastLoadCellMs = 0;

// ---- Serial command buffer ----
static char s_cmdBuffer[32];
static uint8_t s_cmdIndex = 0;

// ---- BLE command buffer ----
static char s_bleCmdBuffer[32];
static uint8_t s_bleCmdIndex = 0;

// Execute a parsed command (shared by Serial and BLE)
void executeCommand(const char* cmd) {
  // Parse command (case-insensitive)
  if (strcasecmp(cmd, "tare") == 0) {
    position::tare();
    if (loadcell::tare()) {
      teleplot::log("CMD: position & load cell tared");
      ble::send("OK: tared\n");
    } else {
      teleplot::log("CMD: position tared, load cell tare failed");
      ble::send("WARN: position tared, load cell tare failed\n");
    }
  } 
  else if (strncasecmp(cmd, "cal ", 4) == 0) {
    // Extract weight value (e.g., "cal 5" for 5 lbs)
    float weight = atof(cmd + 4);
    if (weight > 0.0f) {
      if (loadcell::calibrate(weight)) {
        float factor = loadcell::getCalibration();
        int32_t offset = loadcell::getZeroOffset();
        teleplot::log("CMD: calibration success");
        teleplot::logKV("cal_factor", factor);
        teleplot::logKV("zero_offset", offset);
        ble::sendf("OK: calibrated at %.1f lbs\n", weight);
      } else {
        teleplot::log("CMD: calibration failed (check weight is applied)");
        ble::send("ERROR: calibration failed\n");
      }
    } else {
      teleplot::log("CMD: usage: cal <weight_lbs> (e.g., 'cal 5')");
      ble::send("ERROR: usage: cal <weight_lbs>\n");
    }
  }
  else if (strcasecmp(cmd, "help") == 0) {
    teleplot::log("Commands: tare, cal <lbs>, help, status");
    ble::send("Commands: tare, cal <lbs>, help, status\n");
  }
  else if (strcasecmp(cmd, "status") == 0) {
    teleplot::log("=== Status ===");
    ble::send("=== Status ===\n");
    
    // Load cell status
    int32_t raw;
    if (loadcell::readRaw(raw)) {
      teleplot::logKV("loadcell_raw", raw);
      ble::sendf("loadcell_raw: %ld\n", raw);
    }
    teleplot::logKV("zero_offset", loadcell::getZeroOffset());
    teleplot::logKV("cal_factor", loadcell::getCalibration());
    ble::sendf("zero_offset: %ld\n", loadcell::getZeroOffset());
    ble::sendf("cal_factor: %.2f\n", loadcell::getCalibration());
    ble::sendf("backend: %s\n", loadcell::getBackendName());
    
    float weight;
    if (loadcell::read(weight)) {
      teleplot::logKV("weight_lbs", weight);
      ble::sendf("weight_lbs: %.2f\n", weight);
    } else {
      teleplot::log("weight_lbs: [read failed]");
      ble::send("weight_lbs: [read failed]\n");
    }
    
    // IMU and system status
    teleplot::logKV("imu_ok", imu::imuOk() ? 1 : 0);
    teleplot::logKV("mag_present", imu::magOk() ? 1 : 0);
    teleplot::logKV("ble_connected", ble::isConnected() ? 1 : 0);
    teleplot::logKV("ble_advertising", ble::isAdvertising() ? 1 : 0);
    ble::sendf("imu_ok: %d\n", imu::imuOk() ? 1 : 0);
    ble::sendf("mag_present: %d\n", imu::magOk() ? 1 : 0);
    teleplot::logKV("power_source", battery::isUsbPowered() ? "USB" : "BATTERY");
    teleplot::logKV("battery_status", battery::getStatusString());
  }
  else if (strlen(cmd) > 0) {
    teleplot::log("CMD: unknown (try 'help')");
    ble::send("ERROR: unknown command (try 'help')\n");
  }
}

// Process incoming serial commands (non-blocking)
void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    
    // Handle end of command (newline or carriage return)
    if (c == '\n' || c == '\r') {
      if (s_cmdIndex > 0) {
        s_cmdBuffer[s_cmdIndex] = '\0';  // Null-terminate
        executeCommand(s_cmdBuffer);
        s_cmdIndex = 0;  // Reset buffer
      }
    } 
    else if (s_cmdIndex < sizeof(s_cmdBuffer) - 1) {
      // Add character to buffer (if room)
      s_cmdBuffer[s_cmdIndex++] = c;
    }
  }
}

// Process incoming BLE commands (non-blocking)
void processBleCommands() {
  while (ble::available()) {
    char c = ble::read();
    
    // Handle end of command (newline or carriage return)
    if (c == '\n' || c == '\r') {
      if (s_bleCmdIndex > 0) {
        s_bleCmdBuffer[s_bleCmdIndex] = '\0';  // Null-terminate
        teleplot::log("BLE_CMD: ");
        teleplot::log(s_bleCmdBuffer);
        executeCommand(s_bleCmdBuffer);
        s_bleCmdIndex = 0;  // Reset buffer
      }
    } 
    else if (s_bleCmdIndex < sizeof(s_bleCmdBuffer) - 1) {
      // Add character to buffer (if room)
      s_bleCmdBuffer[s_bleCmdIndex++] = c;
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

  teleplot::log("boot: StrongTrak");

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

  // Initialize load cell
  loadcell::Config lcCfg;
  lcCfg.sckPin = 11;  // Only used for HX711 backend
  lcCfg.dtPin = 12;   // Only used for HX711 backend
  
  teleplot::logKV("loadcell_backend", loadcell::getBackendName());
  
  if (!loadcell::begin(lcCfg)) {
    teleplot::log("WARN: Load cell init failed");
  } else {
    teleplot::log("Load cell ready");
    delay(500);
    if (loadcell::tare()) {
      teleplot::log("Load cell tared");
    } else {
      teleplot::log("WARN: Load cell tare failed");
    }
  }

  teleplot::log("stream:ready");
  teleplot::log("Type 'help' for commands");
}

void loop() {
  const uint32_t nowMs = millis();

  // Process any incoming serial commands (non-blocking)
  processSerialCommands();
  
  // Process any incoming BLE commands (non-blocking)
  processBleCommands();

  // LED heartbeat at 2 Hz.
  digitalWrite(LED_BUILTIN, (nowMs / 250) % 2);

  // Console heartbeat at 1 Hz.
  if ((nowMs - s_lastHeartbeatMs) >= kHeartbeatIntervalMs) {
    s_lastHeartbeatMs = nowMs;
    teleplot::logKV("heartbeat_ms", (int)nowMs);
    
    // Log BLE status periodically (helps debug if serial opened after boot)
    static uint8_t bleStatusCount = 0;
    if (bleStatusCount < 5) {  // Only first 5 heartbeats
      teleplot::logKV("ble_connected", ble::isConnected() ? 1 : 0);
      teleplot::logKV("ble_advertising", ble::isAdvertising() ? 1 : 0);
      bleStatusCount++;
    }
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

  // Load cell reading (at configured rate - with I2C timing)
  static float s_weightLbs = 0.0f;
  if ((nowMs - s_lastLoadCellMs) >= kLoadCellIntervalMs) {
    s_lastLoadCellMs = nowMs;
    float weight;
    if (loadcell::read(weight)) {
      s_weightLbs = weight;
      teleplot::emit("weight_lbs", weight);
    }
    delay(5); // Small delay to let I2C bus settle after load cell read
  }

  // If IMU not available, only heartbeat/status.
  if (!imu::imuOk()) return;

  // Sample at configured rate (with I2C timing).
  if ((nowMs - s_lastSampleMs) < kSampleIntervalMs) return;
  s_lastSampleMs = nowMs;

  delay(2); // Small delay before IMU read to avoid I2C conflicts
  imu::Reading r;
  if (!imu::read(r)) return;

  // Update position tracking
  position::update(r.acc_x, r.acc_y, r.acc_z);
  auto pos = position::getPosition();
  auto vel = position::getVelocity();

  // Emit only Z height (vertical position) - in centimeters for readability
  teleplot::emit("height_cm", pos.z * 100.0f);
  teleplot::emit("vel_z_ms", vel.z);
  
  // Emit raw IMU data for analysis
  teleplot::emit("accel_x_g", r.acc_x);
  teleplot::emit("accel_y_g", r.acc_y);
  teleplot::emit("accel_z_g", r.acc_z);
  teleplot::emit("gyro_x_dps", r.gyro_x);
  teleplot::emit("gyro_y_dps", r.gyro_y);
  teleplot::emit("gyro_z_dps", r.gyro_z);
  teleplot::emit("imu_temp_C", r.temp);
  
  // Magnetometer (optional)
  if (imu::magOk()) {
    teleplot::emitVec3("mag_uT", r.mag_x, r.mag_y, r.mag_z);
  }

  // Send data over BLE at lower rate (to conserve bandwidth)
  if (ble::isConnected() && (nowMs - s_lastBleUpdateMs) >= kBleUpdateIntervalMs) {
    s_lastBleUpdateMs = nowMs;
    
    // CSV format: "height_cm,velocity_m/s,weight_lbs\n"
    // Parsed by Bluetooth_Dashboard web app (src/lib/parser.ts)
    ble::sendf("%.1f,%.2f,%.2f\n", pos.z * 100.0f, vel.z, s_weightLbs);
  }
}
