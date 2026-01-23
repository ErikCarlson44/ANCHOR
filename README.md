# RC Boat Control System

A modern React-based GUI application for controlling and monitoring an RC boat via LORA communication.

![React](https://img.shields.io/badge/React-18.2-61DAFB?logo=react)
![FastAPI](https://img.shields.io/badge/FastAPI-0.109-009688?logo=fastapi)
![Python](https://img.shields.io/badge/Python-3.9+-3776AB?logo=python)

## Features

- 🗺️ **Real-time Map Display** - Interactive Leaflet map with boat tracking and obstacle markers
- 📡 **Animated Radar** - Canvas-based sweeping radar with obstacle blips
- 📊 **Live Telemetry** - GPS, heading, speed, and battery monitoring
- 🎮 **Keyboard Controls** - WASD navigation with visual feedback
- 🔌 **LORA Communication** - Serial port support with simulation mode
- ⚡ **WebSocket Streaming** - 10Hz real-time data updates

## Architecture

```
┌─────────────────┐     WebSocket      ┌─────────────────┐
│   React Frontend │◄──────────────────►│  FastAPI Backend │
│   (Vite + React) │                    │    (Python)      │
└─────────────────┘                    └────────┬────────┘
                                                │
                                           Serial/LORA
                                                │
                                        ┌───────▼───────┐
                                        │   RC Boat     │
                                        └───────────────┘
```

## Quick Start

### Prerequisites

- Python 3.9+
- Node.js 18+
- npm or yarn

### Installation

1. **Clone the repository**

2. **Install Backend Dependencies**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

3. **Install Frontend Dependencies**
   ```bash
   cd frontend
   npm install
   ```

### Running the Application

**Option 1: Run Both Servers**

Terminal 1 - Backend:
```bash
cd backend
python main.py
```

Terminal 2 - Frontend:
```bash
cd frontend
npm run dev
```

**Option 2: Use the start script**
```bash
# Windows
start_app.bat

# Linux/macOS
./start_app.sh
```

Then open your browser to **http://localhost:3000**

## Controls

| Key | Action |
|-----|--------|
| **W** | Throttle Up |
| **S** | Throttle Down |
| **A** | Rudder Left |
| **D** | Rudder Right |
| **SPACE** | All Stop |

Controls respond smoothly with gradual acceleration. Rudder effectiveness increases with speed, just like a real boat.

## API Endpoints

### REST API (Port 8000)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Server status |
| `/ports` | GET | List available COM ports |
| `/connect/{port}` | POST | Connect to COM port |
| `/disconnect` | POST | Disconnect from port |
| `/command` | POST | Send control command |

### WebSocket (Port 8000)

Connect to `ws://localhost:8000/ws` for real-time bidirectional communication.

**Receiving Telemetry:**
```json
{
  "latitude": 29.7604,
  "longitude": -95.3698,
  "heading": 45.0,
  "speed": 5.5,
  "battery": 85.0,
  "obstacles": [
    {"distance": 10.0, "angle": 30.0, "size": 2.0}
  ],
  "timestamp": 1234567890.123
}
```

**Sending Commands:**
```json
{
  "type": "control",
  "throttle": 0.5,
  "steering": -0.3
}
```

## LORA Protocol

### Boat → GUI (Telemetry)
```json
{
  "lat": 29.7604,
  "lon": -95.3698,
  "hdg": 45.0,
  "spd": 5.5,
  "bat": 85.0,
  "obs": [[10.0, 30.0, 2.0], [25.0, -45.0, 1.5]]
}
```

### GUI → Boat (Commands)
```json
{"type": "control", "throttle": 0.5, "steering": -0.3}
{"type": "stop"}
```

## Project Structure

```
SeniorDesignGUI/
├── backend/
│   ├── main.py              # FastAPI server + LORA handler
│   └── requirements.txt     # Python dependencies
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   ├── Header.jsx   # App header with status
│   │   │   ├── Map.jsx      # Leaflet map component
│   │   │   ├── Radar.jsx    # Canvas radar display
│   │   │   ├── Telemetry.jsx# Data display panel
│   │   │   └── Controls.jsx # Keyboard controls panel
│   │   ├── App.jsx          # Main application
│   │   └── main.jsx         # Entry point
│   ├── index.html
│   ├── package.json
│   └── vite.config.js
├── start_app.bat            # Windows start script
└── README.md
```

## Customization

### Change Default Position
Edit `frontend/src/App.jsx`:
```javascript
const [telemetry, setTelemetry] = useState({
  latitude: YOUR_LAT,
  longitude: YOUR_LON,
  // ...
})
```

### Change LORA Baud Rate
Edit `backend/main.py`:
```python
def connect(self, port: str, baudrate: int = 115200):
```

### Adjust Update Rate
Edit `backend/main.py`:
```python
await asyncio.sleep(0.05)  # 20Hz instead of 10Hz
```

## Troubleshooting

### Map Not Loading
- Ensure internet connection (required for map tiles)
- Check browser console for errors

### WebSocket Connection Failed
- Verify backend is running on port 8000
- Check firewall settings

### Serial Port Issues
- Run as administrator on Windows
- Check USB drivers are installed

## Tech Stack

- **Frontend**: React 18, Vite, Framer Motion, React-Leaflet
- **Backend**: FastAPI, uvicorn, pyserial
- **Communication**: WebSocket, JSON
- **Styling**: Custom CSS with CSS Variables

## License

Senior Design Project - Educational Use
