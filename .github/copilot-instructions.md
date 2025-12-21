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
