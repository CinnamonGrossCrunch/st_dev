# StrongTrak Test – Feather Sense IMU Demo

Minimal PlatformIO project for the **Adafruit Feather nRF52840 Sense**.
Streams 6-axis IMU + magnetometer data over USB-serial in Teleplot format.

## Hardware

| Sensor | Part | I²C Address |
|--------|------|-------------|
| IMU (accel + gyro) | LSM6DS3TR-C (or LSM6DS33 on older boards) | 0x6A |
| Magnetometer | LIS3MDL | 0x1C / 0x1E |

The firmware auto-detects the IMU variant via WHO_AM_I.

## Build & Upload

```powershell
# Build
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run

# Build
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run

# Upload - ALWAYS use this command with bootloader prompt
Write-Host "Double-tap the RESET button on your Feather now, then press Enter to upload..."
Read-Host
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload
```

**⚠️ CRITICAL**: Always manually enter bootloader mode before uploading:
1. Double-tap the RESET button on your Feather
2. Red LED should pulse (bootloader mode active)
3. Press Enter when prompted to proceed with upload

## View Data with Teleplot (VS Code)

1. **Close any active PlatformIO Monitor** (only one app can hold the COM port).
2. Open the Command Palette (`Ctrl+Shift+P`) → `Teleplot: Open`.
3. In Teleplot, select **Serial** → `COM3` (or whichever port) → **115200** → **Connect**.
4. Plots appear for `acc_g_x/y/z`, `gyro_dps_x/y/z`, `imu_temp_C`, `mag_uT_x/y/z`.

### RTS/DTR Workaround (if Teleplot shows no data)

On some Windows setups the USB-CDC serial needs **RTS** asserted.
If Teleplot Serial doesn't work, use the UDP bridge instead:

```powershell
Set-Location "c:\Users\Computer\Documents\PlatformIO\Projects\strongtrak_test"
powershell -ExecutionPolicy Bypass -File tools\serial_to_teleplot_udp.ps1 -Port COM3 -Baud 115200 -Timestamp
```

Then just open Teleplot; it listens on UDP 127.0.0.1:47269 automatically.

## Project Structure

```
src/
  main.cpp          – app entry, timing, main loop
  imu.h / .cpp      – sensor init & read abstraction (with EMA smoothing)
  teleplot.h / .cpp – Teleplot serial output helpers
  bleservice.h/.cpp – BLE UART service (Nordic NUS)
  position.h / .cpp – Relative position tracking via IMU integration
tools/
  serial_to_teleplot_udp.ps1 – optional Serial→UDP forwarder
platformio.ini
README.md
```

## Features

### IMU Data Streaming
- **Accelerometer** (±4g), **Gyroscope** (±500 dps), **Magnetometer**, **Temperature**
- EMA smoothing enabled by default (configurable alpha values)
- Auto-detects LSM6DS variants via WHO_AM_I

### Position Tracking
- Relative XYZ position (meters) from tare point
- Double-integration of acceleration with drift compensation
- Auto-zeros velocity when stationary (zero-velocity update)
- **Tares on boot** - current position becomes origin (0,0,0)
- Best for short-term tracking (<10s) or gestures

**Teleplot Channels**: 
- `acc_g_x/y/z`, `gyro_dps_x/y/z`, `imu_temp_C`, `mag_uT_x/y/z`
- `pos_m_x/y/z` (position in meters)
- `vel_ms_x/y/z` (velocity in m/s)

## BLE (Bluetooth Low Energy)

The firmware advertises as "StrongTrak" with a Nordic UART Service (NUS).
Connect with the **nRF Connect** app (iOS/Android) to receive IMU data at 10 Hz.

> **Note**: The file is named `bleservice.h` (not `ble.h`) to avoid conflict 
> with the Nordic SoftDevice SDK's `ble.h` header.

## Extending

- Add new sensor modules following the `imu.h/cpp` pattern.
- Call `teleplot::emit()` / `emitVec3()` for any value you want to plot.
- Use `teleplot::log()` / `logKV()` for console (non-plotted) messages.

## License

MIT – do what you want.
