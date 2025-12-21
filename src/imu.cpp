#include "imu.h"
#include <Wire.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LSM6DSL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ISM330DHCX.h>

namespace imu {

// ---- Static sensor instances (only one will be used) ----
static Adafruit_LSM6DS33 s_imu33;
static Adafruit_LSM6DS3 s_imu3;
static Adafruit_LSM6DSL s_imudsl;
static Adafruit_LSM6DS3TRC s_imu3trc;
static Adafruit_LSM6DSO32 s_imuo32;
static Adafruit_LSM6DSOX s_imux;
static Adafruit_ISM330DHCX s_imuism;
static Adafruit_LSM6DS *s_imu = nullptr;

static Adafruit_LIS3MDL s_mag;

static bool s_imuOk = false;
static bool s_magOk = false;
static const char *s_modelName = nullptr;

// EMA filtering state
static bool s_smoothingEnabled = true;
static SmoothingConfig s_smoothing;
static Reading s_filtered = {0};
static bool s_filterInitialized = false;

// ---- Helpers ----

static bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t *out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  *out = Wire.read();
  return true;
}

static bool tryBegin(Adafruit_LSM6DS &dev, uint8_t addr, const char *model) {
  if (!dev.begin_I2C(addr)) return false;
  s_imu = &dev;
  s_modelName = model;
  return true;
}

// ---- Public API ----

bool begin() {
  // Probe common IMU addresses.
  const uint8_t addrs[] = {0x6A, 0x6B};
  for (uint8_t i = 0; i < sizeof(addrs); i++) {
    const uint8_t addr = addrs[i];
    uint8_t who = 0;
    bool whoOk = i2cRead8(addr, 0x0F, &who);

    // Prefer driver based on WHO_AM_I.
    if (whoOk) {
      if (who == 0x6C && tryBegin(s_imux, addr, "LSM6DSOX")) break;
      if (who == 0x6B && tryBegin(s_imuism, addr, "ISM330DHCX")) break;
      if (who == 0x6A && tryBegin(s_imudsl, addr, "LSM6DSL")) break;
      if (who == 0x69) {
        if (tryBegin(s_imu33, addr, "LSM6DS33")) break;
        if (tryBegin(s_imu3, addr, "LSM6DS3")) break;
      }
    }

    // Brute-force common variants.
    if (tryBegin(s_imux, addr, "LSM6DSOX") ||
        tryBegin(s_imuism, addr, "ISM330DHCX") ||
        tryBegin(s_imudsl, addr, "LSM6DSL") ||
        tryBegin(s_imuo32, addr, "LSM6DSO32") ||
        tryBegin(s_imu3trc, addr, "LSM6DS3TRC") ||
        tryBegin(s_imu33, addr, "LSM6DS33") ||
        tryBegin(s_imu3, addr, "LSM6DS3")) {
      break;
    }
  }

  s_imuOk = (s_imu != nullptr);
  if (s_imuOk) {
    s_imu->setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    s_imu->setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    s_imu->setAccelDataRate(LSM6DS_RATE_104_HZ);
    s_imu->setGyroDataRate(LSM6DS_RATE_104_HZ);
  }

  // Magnetometer (LIS3MDL) at 0x1C or 0x1E.
  s_magOk = s_mag.begin_I2C(0x1C) || s_mag.begin_I2C(0x1E);
  if (s_magOk) {
    s_mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    s_mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    s_mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    s_mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  }

  return s_imuOk;
}

bool imuOk() { return s_imuOk; }
bool magOk() { return s_magOk; }
const char *imuModelName() { return s_modelName; }

void setSmoothing(bool enable, const SmoothingConfig& config) {
  s_smoothingEnabled = enable;
  s_smoothing = config;
  if (!enable) {
    s_filterInitialized = false;  // Reset filter on disable
  }
}

static float applyEMA(float alpha, float newVal, float oldVal) {
  return alpha * newVal + (1.0f - alpha) * oldVal;
}

bool read(Reading &out) {
  if (!s_imuOk) return false;

  sensors_event_t accel, gyro, temp;
  s_imu->getEvent(&accel, &gyro, &temp);

  // Convert to user-friendly units.
  out.acc_x = accel.acceleration.x / 9.80665f;
  out.acc_y = accel.acceleration.y / 9.80665f;
  out.acc_z = accel.acceleration.z / 9.80665f;

  out.gyro_x = gyro.gyro.x * 57.2958f;
  out.gyro_y = gyro.gyro.y * 57.2958f;
  out.gyro_z = gyro.gyro.z * 57.2958f;

  out.temp = temp.temperature;

  if (s_magOk) {
    sensors_event_t m;
    s_mag.getEvent(&m);
    out.mag_x = m.magnetic.x;
    out.mag_y = m.magnetic.y;
    out.mag_z = m.magnetic.z;
  } else {
    out.mag_x = out.mag_y = out.mag_z = 0.0f;
  }

  // Apply EMA smoothing if enabled
  if (s_smoothingEnabled) {
    if (!s_filterInitialized) {
      // First reading: initialize filter state
      s_filtered = out;
      s_filterInitialized = true;
    } else {
      // Apply exponential moving average
      s_filtered.acc_x = applyEMA(s_smoothing.accel, out.acc_x, s_filtered.acc_x);
      s_filtered.acc_y = applyEMA(s_smoothing.accel, out.acc_y, s_filtered.acc_y);
      s_filtered.acc_z = applyEMA(s_smoothing.accel, out.acc_z, s_filtered.acc_z);
      
      s_filtered.gyro_x = applyEMA(s_smoothing.gyro, out.gyro_x, s_filtered.gyro_x);
      s_filtered.gyro_y = applyEMA(s_smoothing.gyro, out.gyro_y, s_filtered.gyro_y);
      s_filtered.gyro_z = applyEMA(s_smoothing.gyro, out.gyro_z, s_filtered.gyro_z);
      
      s_filtered.mag_x = applyEMA(s_smoothing.mag, out.mag_x, s_filtered.mag_x);
      s_filtered.mag_y = applyEMA(s_smoothing.mag, out.mag_y, s_filtered.mag_y);
      s_filtered.mag_z = applyEMA(s_smoothing.mag, out.mag_z, s_filtered.mag_z);
      
      s_filtered.temp = out.temp;  // Don't filter temperature
      
      out = s_filtered;
    }
  }

  return true;
}

}  // namespace imu
