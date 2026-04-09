# ANCHOR - RC Boat Control System

A remote control system for an RC boat with radar obstacle detection, IMU heading, and a web-based GUI.

---

## Quick Start

### 1. Hardware Setup (Pico Wiring)

Connect components to your Raspberry Pi Pico:

| Component | Wire | Pico Pin |
|-----------|------|----------|
| **Radar (RD-03D)** | TX | GP1 |
| | RX | GP0 |
| | VCC | 5V (VBUS) |
| | GND | GND |
| **Servo** | Signal | GP12 |
| | VCC | 5V |
| | GND | GND |
| **IMU (ICM-20948)** | SDA | GP20 |
| | SCL | GP21 |
| | VCC | 3.3V |
| | GND | GND |

### 2. Pico Setup

1. **Install CircuitPython** on your Pico (if not already installed)
   - Download from [circuitpython.org](https://circuitpython.org/board/raspberry_pi_pico/)
   - Hold BOOTSEL button, plug in Pico, drag .uf2 file to RPI-RP2 drive

2. **Install required libraries** in Pico's `/lib` folder:
   - `adafruit_icm20x.mpy`
   - `adafruit_bus_device/` (folder)
   - `adafruit_register/` (folder)
   
   Download from [CircuitPython Libraries](https://circuitpython.org/libraries) - match your CircuitPython version.

3. **Copy `code.py`** to your Pico:
   - Open Thonny IDE
   - Open `Raspberry_PI_PicoScripts/code.py`
   - Save to Pico as `code.py`

4. **Close Thonny** (important - releases the COM port)

### 3. Install GUI Dependencies

**Backend (Python):**
```bash
cd GUI/backend
pip install -r requirements.txt
```

**Frontend (Node.js):**
```bash
cd GUI/frontend
npm install
```

### 4. Run the GUI

Open **two terminals**:

**Terminal 1 - Backend:**
```bash
cd GUI/backend
python main.py
```

**Terminal 2 - Frontend:**
```bash
cd GUI/frontend
npm run dev
```

### 5. Connect to Boat

1. Open browser to **http://localhost:5173**
2. Select the Pico's COM port (e.g., COM5)
3. Click **"Quick Connect"**
4. You should see:
   - Radar display showing detected obstacles
   - Heading from IMU
   - Map with boat position

---

## GUI Controls

### Keyboard Controls
| Key | Action |
|-----|--------|
| W | Throttle up |
| S | Throttle down |
| A | Rudder left |
| D | Rudder right |
| Space | Emergency stop |

### Connection Panel
| Button | Action |
|--------|--------|
| Quick Connect | Connect without handshake (recommended) |
| Connect to Boat | Connect with ping/pong handshake |
| Stop | Pause radar sweep |
| Start | Resume radar sweep |
| Disconnect | Return to simulation mode |

### Radar Panel
| Control | Action |
|---------|--------|
| + / - | Adjust radar range (1-20m) |

---

## Troubleshooting

### "Failed to connect" or "Boat did not respond"
- Make sure **Thonny is closed** (releases COM port)
- Use **Quick Connect** instead of regular Connect
- Check Pico is plugged in via USB

### No radar detections
- Check radar wiring (TX→GP1, RX→GP0, VCC→5V)
- Radar needs 5V power, not 3.3V
- Minimum detection range is ~30cm

### IMU not working
- Check wiring (SDA→GP20, SCL→GP21, VCC→3.3V)
- Verify libraries are in Pico's `/lib` folder
- Check CircuitPython version matches library version

### "No I2C device found"
- SDA/SCL wires may be swapped - try switching them
- Check VCC is connected to 3.3V
- Check GND is connected

---

## Project Structure

```
SeniorDesignGUI/
├── GUI/
│   ├── backend/
│   │   ├── main.py           # FastAPI server
│   │   └── requirements.txt  # Python dependencies
│   └── frontend/
│       ├── src/
│       │   ├── App.jsx       # Main React app
│       │   └── components/   # UI components
│       └── package.json      # Node dependencies
│
├── Raspberry_PI_PicoScripts/
│   └── code.py               # Main Pico code (copy to Pico)
│
├── radar_servo_test/
│   ├── pico_radar_servo.py   # Standalone radar test
│   ├── radar_display.py      # PC radar visualization
│   └── lora_test.py          # LoRa connection test
│
└── README.md                 # This file
```

---

## Data Format

### Telemetry (Pico → GUI)
```json
{
  "lat": 32.7872,
  "lon": -117.2350,
  "hdg": 45.0,
  "spd": 0,
  "bat": 100,
  "obs": [[3.5, 45, 1.0], [2.1, -30, 1.0]]
}
```

| Field | Description |
|-------|-------------|
| lat/lon | GPS coordinates (placeholder until GPS installed) |
| hdg | Heading in degrees (0-360, from IMU) |
| spd | Speed in knots (0 until GPS installed) |
| bat | Battery percentage |
| obs | Obstacles: [distance_m, angle_deg, size] |

### Commands (GUI → Pico)
```json
{"type": "ping"}     // Handshake request
{"type": "stop"}     // Pause radar
{"type": "start"}    // Resume radar
{"type": "control", "throttle": 0.5, "steering": -1}
```

---

## Hardware Requirements

| Component | Model | Notes |
|-----------|-------|-------|
| Microcontroller | Raspberry Pi Pico | Running CircuitPython |
| Radar | RD-03D | 24GHz, 256000 baud |
| IMU | SparkFun ICM-20948 | 9DoF, I2C |
| Servo | SG90 or similar | For radar sweep |
| GPS | (Not yet installed) | UART, 9600 baud |
| LoRa | Adafruit RFM95W | (Not yet installed) |

---

## Future Additions

- [ ] GPS module integration
- [ ] LoRa wireless communication
- [ ] Motor control
- [ ] Battery monitoring
- [ ] Autonomous navigation
