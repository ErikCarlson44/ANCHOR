# Motor Control Test

Test scripts for motor/ESC and rudder servo control.

## Wiring

| Component | Wire | Pico Pin |
|-----------|------|----------|
| **Rudder Servo** | Signal | GP12 |
| | VCC | 5V |
| | GND | GND |
| **Throttle ESC** | Signal | GP13 |
| | VCC | External 5V (battery) |
| | GND | GND (common) |

**Important:** ESC should be powered from your battery, not the Pico. Only the signal wire connects to the Pico. Make sure GND is shared between Pico and ESC.

## Usage

1. Copy `motor_test.py` to Pico as `code.py`
2. Open Thonny serial console
3. Use keyboard to control:

| Key | Action |
|-----|--------|
| R | **Arm motors** (must press first!) |
| W | Throttle up (+10%) |
| S | Throttle down (-10%) |
| A | Rudder full left |
| D | Rudder full right |
| X | Rudder center |
| Q | Stop (zero throttle) |
| Space | Emergency stop (disarms motors) |
| 1 | Throttle 10% |
| 2 | Throttle 25% |
| 3 | Throttle 50% |
| 4 | Throttle 75% |
| 5 | Throttle 100% |

## ESC Arming

The script automatically arms the ESC on startup by:
1. Setting throttle to 0% (1000µs pulse)
2. Waiting 2 seconds

If your ESC doesn't arm, check:
- ESC is powered
- Signal wire connected to GP10
- GND is shared with Pico

## PWM Settings

Standard RC PWM (50Hz, 1000-2000µs):

| Setting | Value |
|---------|-------|
| Frequency | 50 Hz |
| Period | 20 ms |
| Throttle 0% | 1000 µs |
| Throttle 100% | 2000 µs |
| Rudder Left | 1000 µs |
| Rudder Center | 1500 µs |
| Rudder Right | 2000 µs |

Adjust in code if your ESC/servo uses different values.
