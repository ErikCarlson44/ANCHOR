# RC Boat Raspberry Pi Pico Scripts

## Overview

This folder contains the code that runs on the **boat** (Raspberry Pi Pico with CircuitPython).

## File Structure

```
Raspberry_PI_PicoScripts/
├── code.py      <- MAIN FILE (runs on boot, imports all modules)
├── GPS.py       <- GPS module (GPSReader class)
├── LORA.py      <- LORA module (LoraTransceiver class)
├── radar.py     <- Radar module (RadarReader class)
├── motors.py    <- Motor module (MotorController class)
└── README.md
```

| File | Class | Description |
|------|-------|-------------|
| `code.py` | - | **Main controller** - imports and uses all modules |
| `GPS.py` | `GPSReader` | Parses NMEA data from GPS |
| `LORA.py` | `LoraTransceiver` | Sends/receives JSON via RFM9x |
| `radar.py` | `RadarReader` | Reads obstacle data from RD-03D |
| `motors.py` | `MotorController` | Controls rudder servo and ESC |

## Main Script: `code.py`

### What It Does

```
┌─────────────────────────────────────────────────────────────┐
│                    BOAT (Raspberry Pi Pico)                 │
│                                                             │
│   ┌─────────┐     ┌─────────┐     ┌─────────────────────┐  │
│   │   GPS   │────►│         │     │                     │  │
│   └─────────┘     │         │     │    MOTOR CONTROL    │  │
│                   │  MAIN   │────►│  - Rudder (servo)   │  │
│   ┌─────────┐     │ CONTROL │     │  - Throttle (ESC)   │  │
│   │  RADAR  │────►│  LOOP   │     │                     │  │
│   └─────────┘     │         │     └─────────────────────┘  │
│                   │         │                              │
│   ┌─────────┐     │         │     ┌─────────────────────┐  │
│   │ BATTERY │────►│         │◄───►│        LORA         │◄─┼──► GUI
│   └─────────┘     └─────────┘     └─────────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **GPS** → Reads latitude, longitude, speed, heading
2. **Radar** → Detects obstacles (distance, angle)
3. **Battery** → Monitors voltage level
4. **Telemetry Builder** → Combines all data into JSON
5. **LORA TX** → Sends telemetry to GUI (10 Hz)
6. **LORA RX** → Receives commands from GUI
7. **Motor Control** → Applies throttle and steering

### Telemetry Format (Boat → GUI)

```json
{
    "lat": 32.787200,
    "lon": -117.235000,
    "hdg": 45.0,
    "spd": 5.5,
    "bat": 85.0,
    "obs": [[10.5, 30, 1.0], [25.0, -45, 2.0]]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `lat` | float | Latitude (decimal degrees) |
| `lon` | float | Longitude (decimal degrees) |
| `hdg` | float | Heading (0-360°) |
| `spd` | float | Speed (knots) |
| `bat` | float | Battery (0-100%) |
| `obs` | array | Obstacles: [[distance_m, angle_deg, size], ...] |

### Command Format (GUI → Boat)

```json
{"type": "control", "throttle": 0.5, "steering": -1}
{"type": "stop"}
```

| Field | Type | Description |
|-------|------|-------------|
| `type` | string | "control" or "stop" |
| `throttle` | float | 0.0 to 1.0 (0% to 100%) |
| `steering` | int | -1 (left), 0 (center), 1 (right) |

## Hardware Wiring

### Pin Configuration (adjust in `boat_main.py`)

```python
# GPS (UART)
GPS_TX_PIN = board.GP4      # Pico RX ← GPS TX
GPS_RX_PIN = board.GP5      # Pico TX → GPS RX

# Radar (UART)  
RADAR_TX_PIN = board.GP0    # Pico RX ← Radar TX
RADAR_RX_PIN = board.GP1    # Pico TX → Radar RX

# LORA RFM9x (SPI)
LORA_SPI_SCK = board.GP18   # SPI Clock
LORA_SPI_MOSI = board.GP19  # SPI MOSI
LORA_SPI_MISO = board.GP16  # SPI MISO
LORA_CS_PIN = board.GP17    # Chip Select
LORA_RESET_PIN = board.GP20 # Reset

# Motor Control (PWM)
RUDDER_PIN = board.GP12     # Rudder servo
THROTTLE_PIN = board.GP13   # ESC/Throttle

# Battery (ADC)
BATTERY_PIN = board.GP26    # Voltage divider
```

### Wiring Diagram

```
                    ┌─────────────────────┐
                    │   RASPBERRY PI PICO │
                    │                     │
    GPS TX ────────►│ GP4 (UART RX)       │
    GPS RX ◄────────│ GP5 (UART TX)       │
                    │                     │
    Radar TX ──────►│ GP0 (UART RX)       │
    Radar RX ◄──────│ GP1 (UART TX)       │
                    │                     │
    LORA SCK ◄─────►│ GP18 (SPI SCK)      │
    LORA MOSI ◄─────│ GP19 (SPI MOSI)     │
    LORA MISO ─────►│ GP16 (SPI MISO)     │
    LORA CS ◄───────│ GP17 (SPI CS)       │
    LORA RST ◄──────│ GP20 (Reset)        │
                    │                     │
    Rudder Servo ◄──│ GP12 (PWM)          │
    ESC Signal ◄────│ GP13 (PWM)          │
                    │                     │
    Battery ───────►│ GP26 (ADC)          │
    (via divider)   │                     │
                    │                     │
                    │ 3V3 ──► Power       │
                    │ GND ──► Ground      │
                    └─────────────────────┘
```

## Setup Instructions

### 1. Install CircuitPython

1. Download CircuitPython for Pico: https://circuitpython.org/board/raspberry_pi_pico/
2. Hold BOOTSEL button, plug in USB
3. Drag `.uf2` file to RPI-RP2 drive

### 2. Install Libraries

Copy these to `/lib` folder on CIRCUITPY drive:
- `adafruit_rfm9x.mpy`
- `adafruit_bus_device/`

Get them from: https://circuitpython.org/libraries

### 3. Upload Code

1. Copy `boat_main.py` to CIRCUITPY drive
2. Rename to `code.py` (auto-runs on boot)

### 4. Adjust Configuration

Edit the CONFIGURATION section at the top of `boat_main.py`:
- Pin assignments for your wiring
- LORA frequency (915 MHz US, 868 MHz EU)
- Battery voltage range
- Baud rates

## Safety Features

### Failsafe

If no command is received for 0.6 seconds:
- Throttle → 0 (stop)
- Rudder → center
- Motors disarmed

### Arming

- Motors start disarmed (throttle disabled)
- Auto-arms on first throttle command
- Disarms on failsafe or stop command

## Troubleshooting

### No GPS Fix
- Check antenna connection
- Move to open sky (GPS needs satellite view)
- Wait 30-60 seconds for cold start

### LORA Not Communicating
- Verify both modules on same frequency
- Check antenna connections
- Reduce distance for testing
- Verify SPI wiring

### Motors Not Responding
- Check ESC is powered and calibrated
- Verify PWM pins
- Check that motors are armed

### Radar Not Detecting
- Check UART baud rate matches radar
- Verify radar is powered
- Adjust parsing in `RadarParser` for your radar's output format

## Testing Individual Components

```python
# Test GPS only
from GPS import *

# Test LORA only  
from LORA import *

# Test Radar only
from radar_test_with_servo import *
```

## Integration with GUI

The GUI (on your computer) connects via LORA to this boat controller:

```
┌──────────────┐         LORA Radio         ┌──────────────┐
│     GUI      │  ◄─────────────────────►   │     BOAT     │
│  (Computer)  │                            │ (Pi Pico)    │
│              │   Telemetry (JSON) ──────► │              │
│  localhost   │                            │  boat_main.py│
│    :3000     │   ◄────── Commands (JSON)  │              │
└──────────────┘                            └──────────────┘
```

1. Run GUI: `cd frontend && npm run dev`
2. Run Backend: `cd backend && python main.py`
3. Power on boat
4. Select COM port in GUI
5. Click "Connect to Boat"
