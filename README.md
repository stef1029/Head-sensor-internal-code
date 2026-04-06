# Head Sensor — 9-DOF IMU Head Tracking System

Firmware for a head-mounted IMU sensor that outputs real-time orientation (yaw, pitch, roll) at 50 Hz using a DCM (Direction Cosine Matrix) sensor fusion algorithm. Built for Arduino Mega 2560 with a SparkFun 9DOF Sensor Stick.

## Hardware

| Component | Details |
|-----------|---------|
| **Microcontroller** | Arduino Mega 2560 (ATmega2560) |
| **Accelerometer** | ADXL345 — 3-axis, ±16g, I2C `0x53` |
| **Magnetometer** | HMC5883L — 3-axis, I2C `0x1E` |
| **Gyroscope** | MPU-3050 — 3-axis, I2C `0x68` |
| **Sensor Board** | SparkFun 9DOF Sensor Stick (SEN-10724) |

**Pin assignments:**
- **Pin 6** — Sync output (10 ms pulse at 50 Hz for camera/recording synchronisation)
- **Pin 13** — Status LED (on during recording)

## Building & Flashing

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VSCode extension)
- Arduino Mega 2560 connected via USB

### Steps

```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor
pio device monitor --baud 57600
```

> **Note:** Update `upload_port` in `platformio.ini` to match your system's COM port.

## Serial Command Protocol

All commands are sent over serial at **57600 baud**, prefixed with `#`.

### Recording

| Command | Description |
|---------|-------------|
| `#s` | Start recording (LED on, sync pulses begin) |
| `#e` | Stop recording (LED off) |

### Output Mode

| Command | Description |
|---------|-------------|
| `#of` | Request single frame |
| `#ot` | Output angles as text |
| `#ob` | Output angles as binary |
| `#osrt` / `#osrb` | Raw sensor data (text / binary) |
| `#osct` / `#oscb` | Calibrated sensor data (text / binary) |
| `#osbt` / `#osbb` | Both raw + calibrated (text / binary) |
| `#o1` | Enable continuous streaming |
| `#o0` | Disable streaming |

### Calibration & Diagnostics

| Command | Description |
|---------|-------------|
| `#oc` | Enter calibration mode |
| `#on` | Cycle to next calibration sensor |
| `#oe0` | Disable error reporting |
| `#oe1` | Enable error reporting |
| `#oec` | Print error counts |

## Calibration

The firmware supports three-stage interactive calibration:

1. **Accelerometer** — Slowly rotate the sensor through all orientations. Record min/max values for each axis.
2. **Magnetometer** — Rotate the sensor in all directions. Supports extended ellipsoid calibration for hard/soft iron compensation.
3. **Gyroscope** — Keep the sensor stationary for ~10 seconds. The firmware averages the noise offset.

Calibration values are stored as constants in [`globals.h`](include/globals.h). After calibrating, update the `ACCEL_*_MIN/MAX`, `MAGN_*_MIN/MAX`, and `GYRO_AVERAGE_OFFSET_*` values and reflash.

## Output Formats

### Angle Binary Frame

```
STX (0x02) | message_id (4B) | yaw (float) | pitch (float) | roll (float) | ETX (0x03)
```

### Sensor Binary Frame

```
0xAA 0x55 | accel[3] (12B) | magnetom[3] (12B) | gyro[3] (12B)
```

### Text

Human-readable values printed as comma-separated fields.

## Architecture

```
src/
├── main.cpp          # Setup, main loop, command parsing
├── Sensors.cpp       # I2C reads for ADXL345, HMC5883L, MPU-3050
├── DCM.cpp           # Direction Cosine Matrix sensor fusion
├── Compass.cpp       # Tilt-compensated magnetic heading
├── CustomMath.cpp    # Vector/matrix operations
├── Output.cpp        # Binary and text data streaming
└── globals.cpp       # Shared state and calibration constants

include/
├── Sensors.h
├── DCM.h
├── Compass.h
├── CustomMath.h
├── Output.h
└── globals.h         # Calibration values and tuning parameters
```

## Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `DATA_INTERVAL` | 20 ms | Sampling period (50 Hz) |
| `Kp_ROLLPITCH` | 0.02 | Proportional gain, roll/pitch correction |
| `Ki_ROLLPITCH` | 0.00002 | Integral gain, roll/pitch drift |
| `Kp_YAW` | 1.2 | Proportional gain, yaw correction |
| `Ki_YAW` | 0.00002 | Integral gain, yaw drift |

## License

Internal use.
