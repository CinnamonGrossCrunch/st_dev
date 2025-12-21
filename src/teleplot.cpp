#include "teleplot.h"

namespace teleplot {

void begin() {
  // Serial.begin() is expected to be called by the app before this.
}

void emit(const char *name, float value) {
  Serial.print('>');
  Serial.print(name);
  Serial.print(':');
  Serial.println(value);
}

void emitVec3(const char *prefix, float x, float y, float z) {
  Serial.print('>');
  Serial.print(prefix);
  Serial.print("_x:");
  Serial.println(x);

  Serial.print('>');
  Serial.print(prefix);
  Serial.print("_y:");
  Serial.println(y);

  Serial.print('>');
  Serial.print(prefix);
  Serial.print("_z:");
  Serial.println(z);
}

void log(const char *msg) {
  Serial.println(msg);
}

void logKV(const char *key, int value) {
  Serial.print(key);
  Serial.print(':');
  Serial.println(value);
}

void logKV(const char *key, const char *value) {
  Serial.print(key);
  Serial.print(':');
  Serial.println(value);
}

}  // namespace teleplot
