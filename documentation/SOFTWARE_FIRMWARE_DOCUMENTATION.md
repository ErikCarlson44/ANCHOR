# Software/Firmware Development Documentation
## ANCHOR - Autonomous Navigation Control Hub for Oceanic Research

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [GUI / User Control](#2-gui--user-control)
3. [GUI JSON Communication Format](#3-gui-json-communication-format)
4. [GPS Module](#4-gps-module)
5. [IMU Module](#5-imu-module)

---

## 1. System Overview

The ANCHOR system consists of two main components:
- **Shore Station**: React web GUI with FastAPI backend running on a laptop
- **Boat Controller**: Raspberry Pi Pico running CircuitPython with sensor modules

### Complete System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SHORE STATION (Laptop)                             │
│  ┌──────────────────────┐         ┌──────────────────────────────────────┐  │
│  │   React Frontend     │◄───────►│      FastAPI Backend                 │  │
│  │   (localhost:5173)   │WebSocket│      (localhost:8000)                │  │
│  │                      │         │                                      │  │
│  │  ┌────────────────┐  │         │  ┌────────────────────────────────┐  │  │
│  │  │ Map (Leaflet)  │  │         │  │   LORA Handler                 │  │  │
│  │  │ Radar Display  │  │         │  │   - Serial Communication       │  │  │
│  │  │ Telemetry      │  │         │  │   - JSON Encode/Decode         │  │  │
│  │  │ Controls       │  │         │  │   - Simulation Mode            │  │  │
│  │  │ Connection     │  │         │  └────────────────────────────────┘  │  │
│  │  └────────────────┘  │         │              │                       │  │
│  └──────────────────────┘         └──────────────┼───────────────────────┘  │
└──────────────────────────────────────────────────┼──────────────────────────┘
                                                   │ USB Serial
                                    ┌──────────────▼──────────────┐
                                    │   LORA Module (RFM95W)      │
                                    │   915 MHz Transceiver       │
                                    └──────────────┬──────────────┘
                                                   │ RF 915MHz
                                    ┌──────────────▼──────────────┐
                                    │   LORA Module (RFM95W)      │
                                    │   On Boat                   │
                                    └──────────────┬──────────────┘
                                                   │ SPI
┌──────────────────────────────────────────────────┼──────────────────────────┐
│                        RC BOAT (Raspberry Pi Pico)                          │
│  ┌───────────────────────────────────────────────▼───────────────────────┐  │
│  │                         code.py (Main Controller)                      │  │
│  │                                                                        │  │
│  │    ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────────┐   │  │
│  │    │  GPS    │  │  IMU    │  │  Radar  │  │ Battery │  │  Motors  │   │  │
│  │    │ (UART)  │  │ (I2C)   │  │ (UART)  │  │  (ADC)  │  │  (PWM)   │   │  │
│  │    └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬─────┘   │  │
│  │         │            │            │            │            │         │  │
│  │         ▼            ▼            ▼            ▼            ▼         │  │
│  │    ┌─────────────────────────────────────────────────────────────┐   │  │
│  │    │              Telemetry Aggregation (10 Hz)                  │   │  │
│  │    │  {lat, lon, hdg, spd, bat, obs[]} ──► LORA ──► GUI          │   │  │
│  │    └─────────────────────────────────────────────────────────────┘   │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Software Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              DATA FLOW                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   BOAT → GUI (Telemetry - 10Hz)                                            │
│   ─────────────────────────────                                            │
│                                                                             │
│   GPS ──► lat, lon, spd ──┐                                                │
│                           │                                                │
│   IMU ──► hdg, pitch ─────┼──► JSON Packet ──► LORA TX ──► LORA RX        │
│                           │                                                │
│   Radar ──► obstacles ────┤                        │                       │
│                           │                        ▼                       │
│   Battery ──► bat% ───────┘              FastAPI Backend                   │
│                                                    │                       │
│                                                    ▼                       │
│                                             WebSocket (20Hz)               │
│                                                    │                       │
│                                                    ▼                       │
│                                             React Frontend                 │
│                                             (Map, Radar, UI)               │
│                                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   GUI → BOAT (Commands - On Demand)                                        │
│   ─────────────────────────────────                                        │
│                                                                             │
│   Keyboard Input ──► Control State ──► WebSocket ──► FastAPI               │
│   (W/S/A/D/Space)    {throttle,                          │                 │
│                       steering}                          ▼                 │
│                                                    LORA TX                 │
│                                                          │                 │
│                                                          ▼                 │
│                                                    LORA RX (Boat)          │
│                                                          │                 │
│                                                          ▼                 │
│                                                    code.py                 │
│                                                          │                 │
│                                                          ▼                 │
│                                                    Motors Controller       │
│                                                    (Throttle, Rudder)      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. GUI / User Control

### 2.1 Technology Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| Frontend | React 18 + Vite | Modern reactive UI framework |
| Styling | CSS3 + Framer Motion | Animations and responsive design |
| Map | React-Leaflet + OpenStreetMap | Real-time boat position tracking |
| Backend | Python FastAPI | REST API + WebSocket server |
| Communication | WebSocket (20Hz) | Real-time bidirectional data |
| Serial | PySerial | LORA module communication |

### 2.2 GUI Layout

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  ⚓ ANCHOR                                           ● CONNECTED  SIMULATION │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐  ┌───────────────────────────┐  ┌───────────────────┐ │
│  │   TELEMETRY     │  │                           │  │      RADAR        │ │
│  │                 │  │                           │  │                   │ │
│  │ LAT: 32.7872°   │  │                           │  │        ╱╲         │ │
│  │ LON: -117.235°  │  │     INTERACTIVE MAP       │  │       ╱  ╲        │ │
│  │ HDG: 45.0°      │  │                           │  │      ╱ ●  ╲       │ │
│  │ SPD: 5.2 kts    │  │   [Boat Icon with Trail]  │  │     ╱      ╲      │ │
│  │ BAT: 85%        │  │                           │  │    ╱   ●    ╲     │ │
│  │                 │  │                           │  │   ╱──────────╲    │ │
│  ├─────────────────┤  │                           │  │       ▲          │ │
│  │   CONTROLS      │  │                           │  │     [boat]       │ │
│  │                 │  │                           │  │                   │ │
│  │  ┌───────────┐  │  └───────────────────────────┘  ├───────────────────┤ │
│  │  │ THROTTLE  │  │                                 │   CONNECTION      │ │
│  │  │ ████░░░░  │  │                                 │                   │ │
│  │  │ 50%       │  │                                 │ ● SIMULATION MODE │ │
│  │  └───────────┘  │                                 │                   │ │
│  │                 │                                 │ Port: [COM5    ▼] │ │
│  │  ◄──── ○ ────►  │                                 │ [CONNECT TO BOAT] │ │
│  │  RUDDER: CENTER │                                 │                   │ │
│  │                 │                                 │                   │ │
│  │ LIMIT: [10][25] │                                 │                   │ │
│  │ [50][75][100]%  │                                 │                   │ │
│  └─────────────────┘                                 └───────────────────┘ │
├─────────────────────────────────────────────────────────────────────────────┤
│ ● CONNECTED │ MODE: SIMULATION │ UPDATE: 20Hz │ W/S throttle • A/D rudder  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.3 User Control System

#### Control Input State Machine

```
                              ┌─────────────────────┐
                              │    IDLE STATE       │
                              │  throttle = 0       │
                              │  steering = 0       │
                              └──────────┬──────────┘
                                         │
              ┌──────────────────────────┼──────────────────────────┐
              │                          │                          │
              ▼                          ▼                          ▼
    ┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
    │   W KEY HELD    │      │   A KEY HELD    │      │   D KEY HELD    │
    │                 │      │                 │      │                 │
    │ throttle += 0.15│      │ steering = -1   │      │ steering = +1   │
    │ (up to limit)   │      │ (full left)     │      │ (full right)    │
    └─────────────────┘      └─────────────────┘      └─────────────────┘
              │                          │                          │
              │                          ▼                          │
              │              ┌─────────────────┐                    │
              │              │  KEY RELEASED   │                    │
              │              │  steering = 0   │                    │
              │              │  (center)       │                    │
              │              └─────────────────┘                    │
              │                                                     │
              ▼                                                     │
    ┌─────────────────┐                                            │
    │   S KEY HELD    │◄───────────────────────────────────────────┘
    │                 │
    │ throttle -= 0.20│
    │ (down to 0)     │         ┌─────────────────┐
    │                 │         │   SPACE BAR     │
    └────────┬────────┘         │  EMERGENCY STOP │
             │                  │                 │
             │                  │ throttle = 0    │
             ▼                  │ steering = 0    │
    ┌─────────────────┐         └─────────────────┘
    │ W/S RELEASED    │
    │                 │
    │ throttle *= 0.95│  (gradual deceleration)
    │ if < 0.02: = 0  │
    └─────────────────┘
```

#### Control Specifications

| Control | Keys | Behavior | Range |
|---------|------|----------|-------|
| Throttle Up | W | Increase throttle by 15% per tick | 0% to Limit |
| Throttle Down | S | Decrease throttle by 20% per tick | Limit to 0% |
| Rudder Left | A | Binary full left | -1 (100% left) |
| Rudder Right | D | Binary full right | +1 (100% right) |
| Rudder Center | Release A/D | Return to center | 0 |
| Emergency Stop | SPACE | Immediate all-stop | All = 0 |
| Throttle Limit | UI Buttons | Set max throttle | 10%, 25%, 50%, 75%, 100% |

#### Control Update Loop (50ms / 20Hz)

```javascript
// Simplified control logic from App.jsx
useEffect(() => {
  const interval = setInterval(() => {
    let { throttle, steering } = controls
    const maxThrottle = throttleLimit / 100
    
    // Throttle control (forward only, no reverse)
    if (keys.has('w')) {
      throttle = Math.min(maxThrottle, throttle + 0.15)
    } else if (keys.has('s')) {
      throttle = Math.max(0, throttle - 0.2)
    } else {
      throttle *= 0.95  // Gradual deceleration
      if (throttle < 0.02) throttle = 0
    }
    
    // Steering control (binary: -1, 0, +1)
    if (keys.has('a')) steering = -1
    else if (keys.has('d')) steering = 1
    else steering = 0
    
    setControls({ throttle, steering })
    sendCommand(throttle, steering)
  }, 50)  // 20Hz update rate
}, [controls, sendCommand, throttleLimit])
```

### 2.4 Connection State Machine

```
┌───────────────────────────────────────────────────────────────────────────┐
│                        CONNECTION STATE MACHINE                           │
└───────────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────────┐
    │                                                                     │
    │   ┌─────────────────────┐                                          │
    │   │                     │                                          │
    │   │   SIMULATION MODE   │◄──────────────────────────────────────┐  │
    │   │                     │                                       │  │
    │   │ • Simulated data    │  Disconnect /                         │  │
    │   │ • Port selectable   │  Handshake Failed                     │  │
    │   │ • Mission Bay start │                                       │  │
    │   │                     │                                       │  │
    │   └──────────┬──────────┘                                       │  │
    │              │                                                  │  │
    │              │ User clicks "CONNECT TO BOAT"                    │  │
    │              │                                                  │  │
    │              ▼                                                  │  │
    │   ┌─────────────────────┐                                       │  │
    │   │                     │                                       │  │
    │   │     CONNECTING      │                                       │  │
    │   │                     │                                       │  │
    │   │ Step 1: Open Serial │                                       │  │
    │   │   POST /connect/COMx│                                       │  │
    │   │                     │──── Serial Open Failed ───────────────┤  │
    │   │ Step 2: Ping Boat   │                                       │  │
    │   │   POST /handshake   │                                       │  │
    │   │                     │──── Boat No Response (3s timeout) ────┘  │
    │   │ • Animated spinner  │                                          │
    │   │ • "Waiting for boat"│                                          │
    │   │                     │                                          │
    │   └──────────┬──────────┘                                          │
    │              │                                                     │
    │              │ Boat responds with "pong" or telemetry              │
    │              │                                                     │
    │              ▼                                                     │
    │   ┌─────────────────────┐                                          │
    │   │                     │                                          │
    │   │   CONNECTED (LIVE)  │                                          │
    │   │                     │                                          │
    │   │ • Live telemetry    │                                          │
    │   │ • Commands sent     │                                          │
    │   │ • Port locked       │                                          │
    │   │                     │───── User clicks "DISCONNECT" ───────────┘
    │   └─────────────────────┘
    │
    └─────────────────────────────────────────────────────────────────────┘
```

### 2.5 UI Testing Evidence

**Testing performed on actual display:**

1. **Frontend Development Server**: Tested on Chrome browser at 1920x1080 resolution
2. **WebSocket Connection**: Verified 20Hz data streaming in browser developer tools
3. **Keyboard Input**: All control keys (W/A/S/D/Space) tested and responsive
4. **Map Rendering**: Leaflet map with CARTO Dark tiles loads correctly
5. **Radar Visualization**: Canvas-based radar shows obstacles in real-time
6. **Connection Panel**: COM port listing and connection workflow tested

*[Insert screenshots of running GUI here]*

---

## 3. GUI JSON Communication Format

### 3.1 Communication Protocol Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        JSON COMMUNICATION PROTOCOL                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  BOAT → GUI                              GUI → BOAT                         │
│  ──────────                              ──────────                         │
│                                                                             │
│  ┌─────────────────────────┐            ┌─────────────────────────┐        │
│  │  TELEMETRY PACKET       │            │  CONTROL COMMAND        │        │
│  │  (10 Hz from boat)      │            │  (On user input)        │        │
│  │                         │            │                         │        │
│  │  {                      │            │  {                      │        │
│  │    "lat": 32.7872,      │            │    "type": "control",   │        │
│  │    "lon": -117.2350,    │            │    "throttle": 0.75,    │        │
│  │    "hdg": 45.0,         │            │    "steering": -1       │        │
│  │    "spd": 5.2,          │            │  }                      │        │
│  │    "bat": 85,           │            │                         │        │
│  │    "obs": [             │            └─────────────────────────┘        │
│  │      [3.5, 45, 1.2],    │                                               │
│  │      [8.0, -30, 0.8]    │            ┌─────────────────────────┐        │
│  │    ]                    │            │  PING (Handshake)       │        │
│  │  }                      │            │                         │        │
│  │                         │            │  {                      │        │
│  └─────────────────────────┘            │    "type": "ping"       │        │
│                                         │  }                      │        │
│  ┌─────────────────────────┐            │                         │        │
│  │  PONG RESPONSE          │            └─────────────────────────┘        │
│  │  (Handshake reply)      │                                               │
│  │                         │            ┌─────────────────────────┐        │
│  │  {                      │            │  STOP COMMAND           │        │
│  │    "type": "pong"       │            │                         │        │
│  │  }                      │            │  {                      │        │
│  │                         │            │    "type": "stop"       │        │
│  └─────────────────────────┘            │  }                      │        │
│                                         │                         │        │
│                                         └─────────────────────────┘        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Telemetry Packet (Boat → GUI)

Sent at **10 Hz** from the boat via LORA.

```json
{
  "lat": 32.787200,
  "lon": -117.235000,
  "hdg": 45.0,
  "spd": 5.2,
  "bat": 85,
  "obs": [
    [3.5, 45, 1.2],
    [8.0, -30, 0.8]
  ]
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `lat` | float | degrees | Latitude (decimal degrees, + = North) |
| `lon` | float | degrees | Longitude (decimal degrees, + = East) |
| `hdg` | float | degrees | Heading (0-360, 0 = North, clockwise) |
| `spd` | float | knots | Speed over ground |
| `bat` | int | percent | Battery level (0-100%) |
| `obs` | array | mixed | Obstacle array: `[[dist, angle, size], ...]` |

#### Obstacle Array Format

Each obstacle is a 3-element array: `[distance, angle, size]`

| Index | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0 | distance | float | meters | Distance from boat to obstacle |
| 1 | angle | float | degrees | Relative angle (-90 to +90, 0 = ahead) |
| 2 | size | float | meters | Estimated obstacle size/width |

### 3.3 Control Command (GUI → Boat)

Sent on user input via WebSocket → LORA.

```json
{
  "type": "control",
  "throttle": 0.75,
  "steering": -1
}
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `type` | string | "control" | Command type identifier |
| `throttle` | float | 0.0 - 1.0 | Throttle level (0 = stop, 1 = full) |
| `steering` | int | -1, 0, 1 | Rudder position: -1=left, 0=center, 1=right |

### 3.4 Handshake Protocol

```
┌──────────────────────┐                    ┌──────────────────────┐
│       GUI            │                    │       BOAT           │
└──────────┬───────────┘                    └───────────┬──────────┘
           │                                            │
           │  {"type": "ping"}                          │
           │ ──────────────────────────────────────────►│
           │                                            │
           │                      {"type": "pong"}      │
           │ ◄──────────────────────────────────────────│
           │                                            │
           │         Connection Established             │
           │ ◄─────────────────────────────────────────►│
           │                                            │
           │  Telemetry packets begin (10 Hz)           │
           │ ◄──────────────────────────────────────────│
           │                                            │
```

### 3.5 Backend API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Server status check |
| `/ports` | GET | List available COM ports |
| `/connect/{port}` | POST | Open serial connection |
| `/quickconnect/{port}` | POST | Connect without handshake (testing) |
| `/handshake` | POST | Ping boat and wait for response |
| `/disconnect` | POST | Close serial connection |
| `/command` | POST | Send control command |
| `/ws` | WebSocket | Real-time telemetry stream |

### 3.6 Data Validation (Pydantic Models)

```python
# From backend/main.py

class BoatTelemetry(BaseModel):
    latitude: float
    longitude: float
    heading: float
    speed: float
    battery: float
    obstacles: List[dict]
    timestamp: float

class ControlCommand(BaseModel):
    type: str
    throttle: Optional[float] = None
    steering: Optional[float] = None
```

---

## 4. GPS Module

### 4.1 Hardware Status

> **Note**: The GPS module has been ordered but **not yet received**. The following documentation describes the planned integration and tested software implementation.

**Planned Hardware**: Standard UART GPS module with NMEA output (9600 baud)

### 4.2 Software Implementation (Tested with Simulation)

The GPS module (`GPS.py`) has been fully developed and tested using simulated NMEA data.

#### GPS Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           GPS DATA FLOW                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌───────────────────┐                                                     │
│   │   GPS Receiver    │                                                     │
│   │   (UART 9600)     │                                                     │
│   │                   │                                                     │
│   │ NMEA Sentences:   │                                                     │
│   │ $GPGGA,...        │                                                     │
│   │ $GPRMC,...        │                                                     │
│   └─────────┬─────────┘                                                     │
│             │                                                               │
│             │ Serial UART (GP4=RX, GP5=TX)                                  │
│             ▼                                                               │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                      GPSReader Class                                 │   │
│   │                                                                      │   │
│   │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │   │
│   │  │  UART Buffer    │───►│  NMEA Parser    │───►│  GPS Data       │  │   │
│   │  │                 │    │                 │    │                 │  │   │
│   │  │ Raw bytes from  │    │ parse_gga():    │    │ latitude        │  │   │
│   │  │ GPS receiver    │    │   lat, lon, alt │    │ longitude       │  │   │
│   │  │                 │    │   fix, sats     │    │ heading         │  │   │
│   │  │                 │    │                 │    │ speed           │  │   │
│   │  │                 │    │ parse_rmc():    │    │ satellites      │  │   │
│   │  │                 │    │   lat, lon      │    │ fix_quality     │  │   │
│   │  │                 │    │   speed, hdg    │    │ has_fix         │  │   │
│   │  └─────────────────┘    └─────────────────┘    └─────────────────┘  │   │
│   │                                                                      │   │
│   │  update() ──► Reads UART, parses sentences, updates internal state   │   │
│   │  get_data() ──► Returns dictionary with current GPS data             │   │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│             │                                                               │
│             │ get_data() returns:                                           │
│             │ {"lat": 32.7872, "lon": -117.235, "hdg": 45.0, ...}          │
│             ▼                                                               │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                      Main Controller (code.py)                       │   │
│   │                                                                      │   │
│   │    Integrates GPS data into telemetry packet:                        │   │
│   │    telemetry["lat"] = gps.get_data()["lat"]                         │   │
│   │    telemetry["lon"] = gps.get_data()["lon"]                         │   │
│   │    telemetry["spd"] = gps.get_data()["spd"]                         │   │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### Wiring Diagram (Planned)

```
┌─────────────────────────────────────────────────────────────────┐
│                    GPS MODULE WIRING                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   GPS Module                      Raspberry Pi Pico             │
│   ┌─────────┐                     ┌─────────────────┐           │
│   │         │                     │                 │           │
│   │   VCC   │────────────────────►│ 3.3V            │           │
│   │         │                     │                 │           │
│   │   GND   │────────────────────►│ GND             │           │
│   │         │                     │                 │           │
│   │   TX    │────────────────────►│ GP4 (UART0 RX)  │           │
│   │         │                     │                 │           │
│   │   RX    │◄────────────────────│ GP5 (UART0 TX)  │           │
│   │         │                     │                 │           │
│   └─────────┘                     └─────────────────┘           │
│                                                                 │
│   Note: GPS TX connects to Pico RX (GP4)                        │
│         GPS RX connects to Pico TX (GP5)                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### NMEA Parsing Implementation

```python
# From GPS.py - NMEA coordinate conversion
def nmea_deg_to_decimal(raw, hemi):
    """Convert NMEA format (DDMM.MMMM) to decimal degrees."""
    if not raw:
        return None
    v = float(raw)
    deg = int(v // 100)
    minutes = v - deg * 100
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec

# Example: "3247.232" with hemi "N" 
#   deg = 32
#   minutes = 47.232
#   decimal = 32 + 47.232/60 = 32.7872°
```

#### Supported NMEA Sentences

| Sentence | Data Extracted | Description |
|----------|----------------|-------------|
| `$GPGGA` / `$GNGGA` | lat, lon, alt, fix, sats | GPS Fix Data |
| `$GPRMC` / `$GNRMC` | lat, lon, speed, heading | Recommended Minimum |

### 4.3 Integration with Other Components

When the GPS module is received, it will integrate with the system as follows:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      GPS INTEGRATION OVERVIEW                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   1. GPS → Main Controller (code.py)                                        │
│      ────────────────────────────────                                        │
│      - GPSReader.update() called in main loop                               │
│      - Position data aggregated into telemetry packet                       │
│      - GPS heading used as fallback when IMU unavailable                    │
│                                                                             │
│   2. GPS → GUI (via LORA → Backend → WebSocket)                             │
│      ─────────────────────────────────────────────                           │
│      - lat/lon displayed in Telemetry panel                                 │
│      - Position shown on Leaflet map                                        │
│      - Trail rendered showing boat path                                     │
│                                                                             │
│   3. GPS + IMU Sensor Fusion                                                │
│      ────────────────────────────────                                        │
│      - GPS provides position and course-over-ground heading                 │
│      - IMU provides instantaneous heading (more accurate at low speed)      │
│      - code.py prioritizes IMU heading when available:                      │
│                                                                             │
│        if imu:                                                              │
│            telemetry["hdg"] = imu.get_heading()   # IMU preferred           │
│        elif gps and gps.has_fix:                                            │
│            telemetry["hdg"] = gps.get_data()["hdg"]  # GPS fallback         │
│                                                                             │
│   4. GPS Fix Status                                                         │
│      ──────────────────                                                      │
│      - has_fix flag indicates valid position                                │
│      - Debug output shows "FIX" or "NO FIX" status                          │
│      - Telemetry sent regardless (0,0 if no fix)                            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. IMU Module

### 5.1 Hardware

**Module**: SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)

| Specification | Value |
|--------------|-------|
| Accelerometer Range | ±2g to ±16g |
| Gyroscope Range | ±250 to ±2000 dps |
| Magnetometer Range | ±4900 µT |
| Interface | I2C (Qwiic) |
| I2C Address | 0x69 (default) |
| Supply Voltage | 3.3V |

### 5.2 IMU Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           IMU DATA FLOW                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌───────────────────────────────────────────────────────────────────┐     │
│   │                    ICM-20948 IMU Sensor                            │     │
│   │                                                                    │     │
│   │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐               │     │
│   │  │Accelerometer│  │  Gyroscope  │  │Magnetometer │               │     │
│   │  │   (X,Y,Z)   │  │   (X,Y,Z)   │  │   (X,Y,Z)   │               │     │
│   │  │   m/s²      │  │   deg/s     │  │    µT       │               │     │
│   │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘               │     │
│   │         │                │                │                       │     │
│   └─────────┼────────────────┼────────────────┼───────────────────────┘     │
│             │                │                │                             │
│             │ I2C (GP14=SDA, GP15=SCL)        │                             │
│             ▼                ▼                ▼                             │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                      IMUReader Class                                 │   │
│   │                                                                      │   │
│   │  ┌───────────────────────────────────────────────────────────────┐  │   │
│   │  │                    Orientation Calculation                     │  │   │
│   │  │                                                                │  │   │
│   │  │  Pitch = atan2(-ax, sqrt(ay² + az²))                          │  │   │
│   │  │                                                                │  │   │
│   │  │  Roll = atan2(ay, az)                                         │  │   │
│   │  │                                                                │  │   │
│   │  │  ┌─────────────────────────────────────────────────────────┐  │  │   │
│   │  │  │            Tilt-Compensated Heading                      │  │  │   │
│   │  │  │                                                          │  │  │   │
│   │  │  │  mx_comp = mx·cos(pitch) + mz·sin(pitch)                 │  │  │   │
│   │  │  │                                                          │  │  │   │
│   │  │  │  my_comp = mx·sin(roll)·sin(pitch)                       │  │  │   │
│   │  │  │          + my·cos(roll)                                  │  │  │   │
│   │  │  │          - mz·sin(roll)·cos(pitch)                       │  │  │   │
│   │  │  │                                                          │  │  │   │
│   │  │  │  Heading = atan2(-my_comp, mx_comp)                      │  │  │   │
│   │  │  │                                                          │  │  │   │
│   │  │  └─────────────────────────────────────────────────────────┘  │  │   │
│   │  └───────────────────────────────────────────────────────────────┘  │   │
│   │                                                                      │   │
│   │  Output: heading (0-360°), pitch (-90° to +90°), roll (-180° to +180°)  │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.3 Wiring Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    IMU MODULE WIRING                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ICM-20948 (Qwiic)                Raspberry Pi Pico            │
│   ┌─────────────┐                  ┌─────────────────┐          │
│   │             │                  │                 │          │
│   │    3.3V     │─────────────────►│ 3.3V            │          │
│   │             │                  │                 │          │
│   │    GND      │─────────────────►│ GND             │          │
│   │             │                  │                 │          │
│   │    SDA      │◄────────────────►│ GP14 (I2C SDA)  │          │
│   │             │                  │                 │          │
│   │    SCL      │◄────────────────►│ GP15 (I2C SCL)  │          │
│   │             │                  │                 │          │
│   └─────────────┘                  └─────────────────┘          │
│                                                                 │
│   I2C Address: 0x69                                             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 5.4 Tilt-Compensated Heading Algorithm

The IMU provides more accurate heading than GPS at low speeds or when stationary. The implementation uses tilt compensation to correct for boat pitch and roll:

```python
# From IMU.py - Tilt-compensated heading calculation

def update(self):
    # Read raw sensor data
    ax, ay, az = self.imu.acceleration
    mx, my, mz = self.imu.magnetic
    
    # Calculate pitch and roll from accelerometer
    pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / pi
    roll = atan2(ay, az) * 180 / pi
    
    # Tilt compensation for magnetometer
    pitch_rad = radians(pitch)
    roll_rad = radians(roll)
    
    mx_comp = mx * cos(pitch_rad) + mz * sin(pitch_rad)
    my_comp = (mx * sin(roll_rad) * sin(pitch_rad) + 
              my * cos(roll_rad) - 
              mz * sin(roll_rad) * cos(pitch_rad))
    
    # Calculate heading (0-360°)
    heading = atan2(-my_comp, mx_comp) * 180 / pi
    if heading < 0:
        heading += 360
```

### 5.5 Testing Results

The IMU module has been tested on the Raspberry Pi Pico with the following results:

| Test | Status | Notes |
|------|--------|-------|
| I2C Detection | ✅ PASS | ICM-20948 found at 0x69 |
| Accelerometer | ✅ PASS | Gravity vector correctly measured |
| Gyroscope | ✅ PASS | Rotation rates accurate |
| Magnetometer | ✅ PASS | Heading tracks compass direction |
| Tilt Compensation | ✅ PASS | Heading stable during pitch/roll |

**Test Output:**
```
IMU: ICM-20948 initialized (9DoF with magnetometer)
Heading: 45.2  Pitch:  2.1  Roll: -0.8
Heading: 45.5  Pitch:  1.9  Roll: -0.5
Heading: 90.1  Pitch:  3.2  Roll:  1.2  (turned 45° right)
Heading: 135.8  Pitch:  2.8  Roll:  0.9  (turned another 45°)
```

---

## Appendix A: File Structure

```
SeniorDesignGUI/
├── GUI/
│   ├── backend/
│   │   ├── main.py              # FastAPI server
│   │   └── requirements.txt     # Python dependencies
│   │
│   └── frontend/
│       ├── src/
│       │   ├── App.jsx          # Main React component
│       │   ├── App.css          # Global styles
│       │   └── components/
│       │       ├── Map.jsx      # Leaflet map
│       │       ├── Radar.jsx    # Radar visualization
│       │       ├── Telemetry.jsx
│       │       ├── Controls.jsx
│       │       ├── Connection.jsx
│       │       └── Header.jsx
│       └── package.json
│
├── Raspberry_PI_PicoScripts/
│   ├── code.py                  # Main controller
│   ├── GPS.py                   # GPS module
│   ├── IMU.py                   # IMU module
│   ├── LORA.py                  # LORA communication
│   ├── radar.py                 # Radar module
│   └── motors.py                # Motor control
│
└── radar_servo_test/
    ├── pico_radar_servo.py      # Pico radar test code
    ├── radar_display.py         # PC visualization
    └── lora_test.py             # LORA connection test
```

---

## Appendix B: Dependencies

### Python Backend
```
fastapi>=0.100.0
uvicorn>=0.23.0
pyserial>=3.5
websockets>=11.0
pydantic>=2.0
```

### React Frontend
```
react: ^18.2.0
react-leaflet: ^4.2.1
leaflet: ^1.9.4
framer-motion: ^10.12.0
```

### Pico CircuitPython Libraries
```
adafruit_rfm9x.mpy
adafruit_icm20x.mpy
```
