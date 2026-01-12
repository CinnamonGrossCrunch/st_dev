# Copilot Instructions for StrongTrak Project

## CRITICAL: Read These Files First

Before making ANY code changes in this project, you MUST read:

1. **`STRONGTRAK_ARCHITECTURE.txt`** - Contains:
   - Project vision and architecture
   - Phased roadmap with current status
   - Power state machine design
   - Section 18: Agentic Coding Guidelines (CRITICAL for AI assistants)

2. **`PROJECT_SUMMARY.txt`** - Contains:
   - Completed work and lessons learned
   - Challenges overcome (don't repeat mistakes)
   - Current memory/build status

## Quick Reference Rules

1. **BOOTLOADER**: Always prompt user to double-tap RESET before upload
2. **PIO PATH**: Use full path `$env:USERPROFILE\.platformio\penv\Scripts\pio.exe`
3. **NO BLOCKING**: Never use `delay()` in `loop()` - use `millis()` timing
4. **NAMESPACES**: Each module uses its own namespace (imu::, ble::, etc.)
5. **NAMING**: Avoid `ble.h` - conflicts with Nordic SDK
6. **SERIAL**: Windows requires RTS/DTR for USB-CDC serial output
7. **EXPLAIN**: User is a novice embedded programmer - explain jargon

## Current Phase

Check Section 10 of STRONGTRAK_ARCHITECTURE.txt for the phased roadmap.
The project is transitioning from Phase 0 (IMU testbed) to Phase 1 (HX711 load cell).

## User Context

The developer is a capable product/industrial designer but a **novice embedded programmer**.
Provide rationale for decisions and explain acronyms on first use.

## Cross-Project Architecture: Firmware ↔ Web Dashboard

This firmware project is the **source of truth** for BLE data format. A companion web app
consumes the BLE stream:

| Component | Location | Role |
|-----------|----------|------|
| **Firmware (C++)** | `strongtrak_test/` | Runs on Feather nRF52840 Sense. Streams sensor data via BLE NUS (Nordic UART Service). |
| **Web Dashboard (TypeScript/Next.js)** | `Bluetooth_Dashboard/` | Receives BLE stream and displays real-time weight, position, velocity. |

### BLE Data Contract

The firmware sends CSV lines over BLE NUS TX characteristic:

```
height_cm,velocity_m/s,weight_lbs\n
```

Example: `12.5,0.34,45.67\n`

**Parser location:** `Bluetooth_Dashboard/src/lib/parser.ts`

### Sync Rules

1. **Firmware is source of truth** - If you change the BLE data format in `main.cpp`, you MUST update `parser.ts` to match.
2. **Service UUID** - Both use Nordic UART Service: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
3. **Device name** - Firmware advertises as `"StrongTrak"` (set in `ble::init()`)
4. **Test locally** - Run dashboard on `localhost:3000` (Web Bluetooth works on localhost)
