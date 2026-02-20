# Radar & Servo Test - ANCHOR Project

Test code for RD-03D radar and LoRa communication.

## Files

| File | Purpose | Runs On |
|------|---------|---------|
| `pico_radar_servo.py` | Radar + servo sweep | Raspberry Pi Pico |
| `radar_display.py` | 360° radar visualization | PC (Python) |
| `lora_test.py` | LoRa connection test | Raspberry Pi Pico |

---

## Radar + Servo Test

### Hardware Required

| Component | Description |
|-----------|-------------|
| Raspberry Pi Pico | CircuitPython |
| RD-03D Radar | 24GHz radar module |
| Servo Motor | SG90 or similar |

### Wiring

```
          ┌─────────────────────────┐
          │   RASPBERRY PI PICO     │
          │                         │
Radar TX ─┼─► GP1 (UART0 RX)        │
Radar RX ─┼─◄ GP0 (UART0 TX)        │
          │                         │
Servo    ─┼─◄ GP12 (PWM)            │
          │                         │
          │ VBUS (5V) → Radar VCC   │
          │ VBUS (5V) → Servo VCC   │
          │ GND       → Common GND  │
          └─────────────────────────┘
```

### Usage

1. Copy `pico_radar_servo.py` to Pico as `code.py`
2. Close Thonny
3. Install PC requirements: `pip install pyserial`
4. Run display: `python radar_display.py COM5`

### Display Controls

| Key | Action |
|-----|--------|
| `+` / `=` | Increase range |
| `-` | Decrease range |
| `ESC` | Quit |

---

## LoRa Test

Tests Adafruit RFM95W LoRa module connection.

### Hardware

| Component | Description |
|-----------|-------------|
| Raspberry Pi Pico | CircuitPython |
| Adafruit RFM95W | 915 MHz LoRa breakout |

### Wiring

```
RFM95W      Pico
──────────────────
VIN    →    3.3V
GND    →    GND
SCK    →    GP18 (SPI0 SCK)
MISO   →    GP16 (SPI0 RX)
MOSI   →    GP19 (SPI0 TX)
CS     →    GP17
RST    →    GP20
G0     →    GP21
```

### Usage

1. Copy `lora_test.py` to Pico as `code.py`
2. Open Thonny to see output
3. Check results for SUCCESS/FAIL

---

## Radar Technical Notes

**RD-03D Frame Format:**
```
AA FF [type] [target1 x8] [target2 x8] [target3 x8] 55 CC
```

- Baud rate: 256000
- Up to 3 simultaneous targets
- Values in mm (signed 16-bit, bit 15 = sign)
- Min range: ~30cm, Max range: ~12m
