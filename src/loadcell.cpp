// loadcell.cpp - Load cell façade that delegates to selected backend
// Backend is selected at compile time via LOADCELL_BACKEND_* defines

#include "loadcell.h"
#include "loadcell_backend.h"

namespace loadcell {

bool begin(const Config& cfg) {
  // Convert public Config to backend config
  loadcell_backend::BackendConfig backendCfg;
  backendCfg.sckPin = cfg.sckPin;
  backendCfg.dtPin = cfg.dtPin;
  backendCfg.calibrationFactor = cfg.calibrationFactor;
  backendCfg.zeroOffset = cfg.zeroOffset;
  
  return loadcell_backend::init(backendCfg);
}

bool readRaw(int32_t& value) {
  return loadcell_backend::readRaw(value);
}

bool read(float& weightLbs) {
  return loadcell_backend::read(weightLbs);
}

bool tare(uint8_t samples) {
  return loadcell_backend::tare(samples);
}

bool calibrate(float knownWeightLbs, uint8_t samples) {
  return loadcell_backend::calibrate(knownWeightLbs, samples);
}

void setCalibration(float factor) {
  loadcell_backend::setCalibration(factor);
}

float getCalibration() {
  return loadcell_backend::getCalibration();
}

int32_t getZeroOffset() {
  return loadcell_backend::getZeroOffset();
}

bool isReady() {
  return loadcell_backend::isReady();
}

const char* getBackendName() {
#if defined(LOADCELL_BACKEND_NAU7802)
  return "NAU7802";
#else
  return "HX711";
#endif
}

}  // namespace loadcell
