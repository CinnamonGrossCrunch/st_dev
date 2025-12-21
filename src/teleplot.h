#pragma once
// teleplot.h - Teleplot serial output helpers
// Emits Teleplot-serial format: >name:value\n (plotted) or plain text (console).

#include <Arduino.h>

namespace teleplot {

// Initialize (currently just ensures Serial is ready).
void begin();

// Emit a plottable value: >name:value
void emit(const char *name, float value);

// Emit a 3D vector as three plotted values: >prefix_x:..., >prefix_y:..., >prefix_z:...
void emitVec3(const char *prefix, float x, float y, float z);

// Emit a console log (no leading >, won't be plotted).
void log(const char *msg);

// Emit a key:value console log (no leading >).
void logKV(const char *key, int value);
void logKV(const char *key, const char *value);

}  // namespace teleplot
