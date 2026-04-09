"""
FastAPI Backend for RC Boat Control System
Handles LORA communication and WebSocket streaming to React frontend.
"""

import asyncio
import json
import math
import random
import time
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional, Set, Tuple

import serial
import serial.tools.list_ports
from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, field_validator


# ============== Data Models ==============

class BoatTelemetry(BaseModel):
    latitude: float
    longitude: float
    heading: float
    speed: float
    battery: float
    satellites: int = 0
    obstacles: List[dict]
    timestamp: float


class ControlCommand(BaseModel):
    type: str
    throttle: Optional[float] = None
    steering: Optional[float] = None

    @field_validator("throttle", "steering", mode="before")
    @classmethod
    def no_bool_control(cls, v: Any) -> Any:
        if isinstance(v, bool):
            return None
        return v


def coerce_throttle_01(v: Any) -> float:
    """Match Pico parse_throttle_01: 0..1, or 1..100 as percent; bool already stripped by model."""
    if v is None:
        return 0.0
    try:
        x = float(v)
    except (TypeError, ValueError):
        return 0.0
    if isinstance(x, float) and not math.isfinite(x):
        return 0.0
    if x > 1.0:
        if x <= 100.0:
            x = x / 100.0
        else:
            return 0.0
    return max(0.0, min(1.0, x))


def coerce_steering_pm1(v: Any) -> float:
    if v is None:
        return 0.0
    try:
        x = float(v)
    except (TypeError, ValueError):
        return 0.0
    if isinstance(x, float) and not math.isfinite(x):
        return 0.0
    return max(-1.0, min(1.0, x))


def json_safe_telemetry(telemetry: BoatTelemetry) -> Dict[str, Any]:
    """
    WebSocket / browser JSON must not contain NaN or Infinity (invalid in ECMAScript;
    JSON.parse throws and the React app can go blank).
    """
    d = telemetry.model_dump()

    def fix_float(v: Any, default: float = 0.0) -> float:
        try:
            x = float(v)
        except (TypeError, ValueError):
            return default
        return default if isinstance(x, float) and not math.isfinite(x) else x

    d["latitude"] = fix_float(d.get("latitude"), 32.7872)
    d["longitude"] = fix_float(d.get("longitude"), -117.2350)
    d["heading"] = fix_float(d.get("heading"), 0.0)
    d["speed"] = fix_float(d.get("speed"), 0.0)
    d["battery"] = fix_float(d.get("battery"), 100.0)
    d["timestamp"] = fix_float(d.get("timestamp"), time.time())

    try:
        d["satellites"] = int(d.get("satellites") or 0)
    except (TypeError, ValueError):
        d["satellites"] = 0

    obs = d.get("obstacles")
    if not isinstance(obs, list):
        obs = []
    clean_obs = []
    for item in obs:
        if not isinstance(item, dict):
            continue
        row = {}
        for ok, ov in item.items():
            if isinstance(ov, float) and not math.isfinite(ov):
                row[ok] = 0.0
            elif isinstance(ov, (int, float, str, bool)) or ov is None:
                row[ok] = ov
            else:
                row[ok] = ov
        clean_obs.append(row)
    d["obstacles"] = clean_obs
    return d


# ============== LORA Handler ==============

class LoraHandler:
    def __init__(self):
        self.serial_connection: Optional[serial.Serial] = None
        self.simulation_mode = True
        self.boat_connected = False  # True when boat responds to ping
        self.connected_clients: Set[WebSocket] = set()
        
        # Simulation state - Mission Bay, San Diego
        self._sim_lat = 32.7872
        self._sim_lon = -117.2350
        self._sim_heading = 45.0
        self._sim_speed = 0.0
        self._sim_throttle = 0.0
        self._sim_steering = 0.0
        self._sim_battery = 100.0
        
        # Simulated obstacles (persistent)
        self._sim_obstacles = []
        self._sim_obstacle_update_time = 0
        self._sim_obstacle_duration = 3.0  # Obstacles persist for 3 seconds
        
        # Last known live telemetry (used when no new data available)
        self._last_live_telemetry = None
        
        # "json" = Pico / ground bridge (newline JSON). "mdot" = MultiTech mDot AT firmware (Micro UDK).
        self.modem_type: str = "json"
        
    def list_ports(self) -> List[str]:
        """List available COM ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port: str, baudrate: int = 9600, modem: str = "json") -> bool:
        """Connect to serial (Pico JSON bridge or mDot AT on Micro UDK)."""
        try:
            if self.serial_connection:
                self.serial_connection.close()
            # rtscts/dsrdtr False: avoid Windows USB-UART toggling RTS/DTR on open (can reset mDot)
            self.serial_connection = serial.Serial(
                port,
                baudrate,
                timeout=0.5,
                write_timeout=2,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self.modem_type = modem if modem in ("json", "mdot") else "json"
            if self.modem_type == "mdot":
                self._mdot_set_line_levels(dtr=True, rts=False)
                time.sleep(0.4)
                if baudrate not in (115200, 57600, 9600, 38400, 19200):
                    print(
                        f"mDot: unusual baud {baudrate} — try 9600 or 115200, or GET /probe/mdot?port=..."
                    )
            # Don't set simulation_mode=False yet - wait for handshake
            return True
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            return False

    def _mdot_set_line_levels(self, dtr: bool, rts: bool) -> None:
        """Set RS-232 control lines (best-effort; ignored if unsupported)."""
        if not self.serial_connection or not self.serial_connection.is_open:
            return
        try:
            self.serial_connection.dtr = dtr
            self.serial_connection.rts = rts
        except (AttributeError, serial.SerialException):
            pass
    
    def ping_boat(self, timeout: float = 3.0) -> bool:
        """
        Handshake: JSON ping for Pico bridge, or AT for MultiTech mDot.
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            return False
        if self.modem_type == "mdot":
            return self._ping_mdot(timeout=timeout)
        return self._ping_json_bridge(timeout)
    
    def _ping_json_bridge(self, timeout: float = 3.0) -> bool:
        """Send JSON ping; expect pong or telemetry with lat."""
        try:
            self.serial_connection.reset_input_buffer()
            ping_msg = json.dumps({"type": "ping"}) + "\n"
            self.serial_connection.write(ping_msg.encode())
            print("Sent JSON ping to boat...")
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if self.serial_connection.in_waiting > 0:
                    try:
                        line = self.serial_connection.readline().decode().strip()
                        if line:
                            data = json.loads(line)
                            if data.get("type") == "pong" or "lat" in data:
                                print("Boat responded (JSON)!")
                                self.boat_connected = True
                                self.simulation_mode = False
                                return True
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        pass
                time.sleep(0.05)
            print(f"Boat did not respond within {timeout}s")
            return False
        except serial.SerialException as e:
            print(f"Ping failed: {e}")
            return False
    
    def _ping_mdot(self, timeout: float = 8.0) -> bool:
        """
        MultiTech mDot on Micro UDK: send AT commands, expect OK (or identifiable ATI text).
        Does not use LoRaWAN join — only verifies UART + AT firmware alive.
        """
        def _raw_indicates_success(raw: bytes) -> bool:
            u = raw.upper()
            if b"OK" in u:
                return True
            if b"ERROR" in u:
                return True
            low = raw.lower()
            if b"multitech" in low or b"mdot" in low or b"mt-dot" in low:
                return True
            try:
                s = raw.decode("utf-8", errors="ignore")
            except Exception:
                s = ""
            if "915" in s and any(c.isalnum() for c in s):
                return True
            return False

        def _looks_like_mdot_response(text: str) -> bool:
            return _raw_indicates_success(text.encode(errors="replace"))

        def _drain_bytes(duration_s: float) -> bytes:
            out = bytearray()
            t0 = time.time()
            while time.time() - t0 < duration_s:
                n = self.serial_connection.in_waiting
                if n:
                    out += self.serial_connection.read(n)
                time.sleep(0.02)
            return bytes(out)

        def _try_at_command(cmd: bytes, subtimeout: float) -> Tuple[bool, str]:
            self.serial_connection.reset_input_buffer()
            time.sleep(0.05)
            self.serial_connection.write(cmd)
            self.serial_connection.flush()
            raw = bytearray()
            start = time.time()
            while time.time() - start < subtimeout:
                n = self.serial_connection.in_waiting
                if n:
                    raw += self.serial_connection.read(n)
                    if _raw_indicates_success(bytes(raw)):
                        return True, bytes(raw).decode(errors="replace")
                time.sleep(0.02)
            tail = _drain_bytes(0.08)
            if tail:
                raw += tail
                if _raw_indicates_success(bytes(raw)):
                    return True, bytes(raw).decode(errors="replace")
            return False, bytes(raw).decode(errors="replace")

        def _rx_is_only_null_or_noise(s: str) -> bool:
            stripped = s.replace("\x00", "").replace("\xff", "").strip()
            return len(stripped) < 2

        def _format_rx_debug(s: str) -> str:
            b = s.encode("latin-1", errors="replace")
            hx = b[:48].hex()
            return f"{s!r} | hex[:48]={hx}"

        try:
            br = getattr(self.serial_connection, "baudrate", "?")
            print(f"mDot handshake: port={self.serial_connection.port!r} baud={br}")

            def _run_at_sequence(budget: float) -> Tuple[bool, str]:
                self.serial_connection.reset_input_buffer()
                self.serial_connection.reset_output_buffer()
                time.sleep(0.35)
                pre_b = _drain_bytes(0.25)
                pre = pre_b.decode(errors="replace")
                if pre_b and _raw_indicates_success(pre_b):
                    return True, pre

                per_cmd_budget = max(1.5, min(3.0, budget / 2))
                commands = (
                    b"AT\r\n",
                    b"AT\r",
                    b"ATI\r\n",
                    b"ATI\r",
                )
                last_local = pre
                deadline = time.time() + budget
                for cmd in commands:
                    if time.time() > deadline:
                        break
                    remain = max(0.5, deadline - time.time())
                    ok, chunk = _try_at_command(cmd, min(per_cmd_budget, remain))
                    last_local += chunk
                    if ok:
                        return True, last_local
                extra = _drain_bytes(0.35).decode(errors="replace")
                last_local += extra
                if extra and _looks_like_mdot_response(last_local):
                    return True, last_local
                return False, last_local

            line_profiles = (
                (True, False),
                (False, False),
                (True, True),
            )
            last_buf = ""
            profile_budget = max(3.0, timeout / len(line_profiles))
            for dtr, rts in line_profiles:
                self._mdot_set_line_levels(dtr=dtr, rts=rts)
                time.sleep(0.2)
                ok, last_buf = _run_at_sequence(profile_budget)
                if ok:
                    print("mDot responded (AT / ATI)")
                    self.boat_connected = True
                    self.simulation_mode = False
                    self._last_live_telemetry = self._mdot_placeholder_telemetry()
                    return True
                if last_buf and not _rx_is_only_null_or_noise(last_buf):
                    break

            snippet = _format_rx_debug(last_buf[:200])
            print(
                "mDot: no OK seen (try other baud e.g. 57600, swap TX/RX, or confirm UDK USB driver). "
                f"Last RX ({len(last_buf)} chars): {snippet}"
            )
            return False
        except serial.SerialException as e:
            print(f"mDot AT ping failed: {e}")
            return False
    
    def _mdot_placeholder_telemetry(self) -> BoatTelemetry:
        """Until LoRaWAN / +EVT parsing is wired, show connected state with default map position."""
        return BoatTelemetry(
            latitude=self._sim_lat,
            longitude=self._sim_lon,
            heading=0.0,
            speed=0.0,
            battery=100.0,
            satellites=0,
            obstacles=[],
            timestamp=time.time(),
        )
    
    def disconnect(self):
        """Disconnect from LORA module."""
        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None
        self.simulation_mode = True
        self.boat_connected = False
        self.modem_type = "json"
    
    def send_command(self, command: ControlCommand):
        """Send control command to boat."""
        t_cmd = 0.0
        s_cmd = 0.0
        if command.type == "control":
            t_cmd = coerce_throttle_01(command.throttle)
            s_cmd = coerce_steering_pm1(command.steering)
            self._sim_throttle = t_cmd
            self._sim_steering = s_cmd
        elif command.type == "stop":
            self._sim_throttle = 0.0
            self._sim_steering = 0.0
        
        if self.serial_connection and self.serial_connection.is_open:
            if self.modem_type == "mdot":
                # AT firmware does not accept JSON control strings here
                return
            if command.type == "control":
                cmd_str = (
                    json.dumps({"type": "control", "throttle": t_cmd, "steering": s_cmd}, separators=(",", ":"))
                    + "\n"
                )
            else:
                cmd_str = json.dumps(command.model_dump(exclude_none=True), separators=(",", ":")) + "\n"
            self.serial_connection.write(cmd_str.encode())
            self.serial_connection.flush()
    
    def get_telemetry(self) -> BoatTelemetry:
        """Get current telemetry (from serial or simulation)."""
        if self.simulation_mode:
            return self._generate_simulation()
        if self.modem_type == "mdot":
            return self._read_serial_mdot()
        return self._read_serial()
    
    def _read_serial_mdot(self) -> BoatTelemetry:
        """Drain mDot UART; use placeholder until +EVT / JSON bridge is implemented."""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.simulation_mode = True
            return self._generate_simulation()
        try:
            if self.serial_connection.in_waiting:
                _ = self.serial_connection.read(self.serial_connection.in_waiting)
                # Future: parse +EVT lines for payload / network data
            if self._last_live_telemetry:
                t = self._last_live_telemetry
                return BoatTelemetry(
                    latitude=t.latitude,
                    longitude=t.longitude,
                    heading=t.heading,
                    speed=t.speed,
                    battery=t.battery,
                    satellites=t.satellites,
                    obstacles=list(t.obstacles) if t.obstacles else [],
                    timestamp=time.time(),
                )
            return self._mdot_placeholder_telemetry()
        except (serial.SerialException, UnicodeDecodeError):
            pass
        if self._last_live_telemetry:
            return self._last_live_telemetry
        return self._mdot_placeholder_telemetry()
    
    def _read_serial(self) -> Optional[BoatTelemetry]:
        """Read telemetry from serial port (JSON / Pico)."""
        if not self.serial_connection or not self.serial_connection.is_open:
            # Connection lost - fall back to simulation
            self.simulation_mode = True
            return self._generate_simulation()
        
        try:
            if self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode().strip()
                # Skip non-telemetry messages (like pong)
                if not line.startswith('{') or '"type"' in line:
                    if self._last_live_telemetry:
                        return self._last_live_telemetry
                    return self._generate_empty_telemetry()
                
                data = json.loads(line)
                
                # Only process actual telemetry (has 'lat' field)
                if 'lat' not in data:
                    if self._last_live_telemetry:
                        return self._last_live_telemetry
                    return self._generate_empty_telemetry()
                
                # Parse obstacles - handle both old [dist, angle, size] and new [dist, sector] formats
                obstacles = []
                for o in data.get('obs', []):
                    if not isinstance(o, (list, tuple)) or len(o) < 2:
                        continue
                    dist = o[0]
                    if o[1] in (0, 1, 2) and len(o) == 2:
                        sector_to_angle = {0: 60, 1: 0, 2: -60}
                        angle = sector_to_angle.get(o[1], 0)
                        obstacles.append({'distance': dist, 'angle': angle, 'size': 1.0})
                    else:
                        obstacles.append({
                            'distance': dist,
                            'angle': o[1],
                            'size': o[2] if len(o) > 2 else 1.0
                        })
                
                telemetry = BoatTelemetry(
                    latitude=data.get('lat', 0),
                    longitude=data.get('lon', 0),
                    heading=data.get('hdg', 0),
                    speed=data.get('spd', 0),
                    battery=data.get('bat', 100),
                    satellites=data.get('sats', 0),
                    obstacles=obstacles,
                    timestamp=time.time()
                )
                self._last_live_telemetry = telemetry
                return telemetry
        except (json.JSONDecodeError, serial.SerialException, UnicodeDecodeError):
            pass
        except Exception:
            pass

        # No new data - return last known telemetry (not simulation!)
        if self._last_live_telemetry:
            return self._last_live_telemetry
        
        return self._generate_empty_telemetry()
    
    def _generate_empty_telemetry(self) -> BoatTelemetry:
        """Return empty telemetry when no data available."""
        return BoatTelemetry(
            latitude=self._sim_lat,
            longitude=self._sim_lon,
            heading=0,
            speed=0,
            battery=100,
            satellites=0,
            obstacles=[],
            timestamp=time.time()
        )
    
    def _generate_simulation(self) -> BoatTelemetry:
        """Generate simulated telemetry."""
        # Update heading based on steering (binary: -1, 0, or 1)
        # Rudder effect scales with speed - need forward motion to turn
        if self._sim_steering != 0 and self._sim_speed > 0.5:
            turn_rate = 4 * self._sim_steering * (self._sim_speed / 12)
            self._sim_heading = (self._sim_heading + turn_rate) % 360
        
        # Update speed based on throttle (0 to max, no reverse)
        target_speed = max(0, self._sim_throttle) * 12
        self._sim_speed += (target_speed - self._sim_speed) * 0.3  # Faster response
        self._sim_speed = max(0, self._sim_speed)  # No negative speed
        
        # Move boat
        if abs(self._sim_speed) > 0.1:
            heading_rad = math.radians(self._sim_heading)
            speed_factor = self._sim_speed * 0.000005
            self._sim_lat += math.cos(heading_rad) * speed_factor
            self._sim_lon += math.sin(heading_rad) * speed_factor
        
        # Add some drift
        self._sim_lat += random.uniform(-0.000002, 0.000002)
        self._sim_lon += random.uniform(-0.000002, 0.000002)
        
        # Battery drain
        self._sim_battery = max(0, self._sim_battery - 0.001)
        
        # Generate persistent obstacles (update every few seconds)
        current_time = time.time()
        if current_time - self._sim_obstacle_update_time > self._sim_obstacle_duration:
            self._sim_obstacle_update_time = current_time
            # Generate new obstacles - one per sector randomly
            self._sim_obstacles = []
            # Randomly decide which sectors have obstacles
            for sector in range(3):  # 0=right, 1=center, 2=left
                if random.random() > 0.4:  # 60% chance per sector
                    # Convert sector to angle: 0=right(60°), 1=center(0°), 2=left(-60°)
                    sector_to_angle = {0: 60, 1: 0, 2: -60}
                    self._sim_obstacles.append({
                        'distance': random.randint(1, 6),
                        'angle': sector_to_angle[sector],
                        'size': 1.0
                    })
        
        return BoatTelemetry(
            latitude=self._sim_lat,
            longitude=self._sim_lon,
            heading=self._sim_heading,
            speed=abs(self._sim_speed),
            battery=self._sim_battery,
            satellites=8,
            obstacles=self._sim_obstacles,
            timestamp=time.time()
        )


MDOT_PROBE_BAUDS = [115200, 57600, 38400, 19200, 9600]


def probe_mdot_uart(port: str) -> List[dict]:
    """Open `port` fresh at each baud, send AT\\r\\n, collect raw RX (does not use LoraHandler)."""
    results: List[dict] = []
    for baud in MDOT_PROBE_BAUDS:
        row: dict = {
            "baud": baud,
            "rx_bytes": 0,
            "rx_hex": "",
            "ascii_preview": "",
            "at_ok": False,
            "error": None,
        }
        ser: Optional[serial.Serial] = None
        try:
            ser = serial.Serial(
                port,
                baud,
                timeout=0.3,
                write_timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            ser.dtr = True
            ser.rts = False
            time.sleep(0.25)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(b"AT\r\n")
            ser.flush()
            raw = bytearray()
            t0 = time.time()
            while time.time() - t0 < 1.2:
                n = ser.in_waiting
                if n:
                    raw += ser.read(n)
                u = bytes(raw).upper()
                if b"OK" in u or b"ERROR" in u:
                    break
                time.sleep(0.02)
            if ser.in_waiting:
                raw += ser.read(ser.in_waiting)
            buf = bytes(raw)
            row["rx_bytes"] = len(buf)
            row["rx_hex"] = buf[:64].hex()
            row["ascii_preview"] = (
                buf[:128].decode("ascii", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
            )
            u = buf.upper()
            row["at_ok"] = (
                b"OK" in u
                or b"ERROR" in u
                or b"MULTITECH" in u
                or b"MDOT" in u
            )
        except serial.SerialException as e:
            row["error"] = str(e)
        finally:
            if ser is not None and ser.is_open:
                ser.close()
        results.append(row)
    return results


# ============== FastAPI App ==============

lora = LoraHandler()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("=== RC Boat Control Backend Starting ===")
    print("mDot probe: GET /probe/mdot?port=COM4  |  GET /probe/mdot/COM4")
    yield
    # Shutdown
    lora.disconnect()
    print("Backend shutdown complete.")


app = FastAPI(
    title="RC Boat Control API",
    description="Backend for RC Boat Control System",
    lifespan=lifespan
)

# CORS for React frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    return {"status": "online", "simulation": lora.simulation_mode}


@app.get("/ports")
async def get_ports():
    """Get available COM ports."""
    return {"ports": lora.list_ports()}


@app.get("/probe/mdot")
async def probe_mdot_via_query(port: str = Query(..., description="Windows COM port, e.g. COM4")):
    """Example: `http://localhost:8000/probe/mdot?port=COM4`"""
    return await _probe_mdot_impl(port)


@app.get("/probe/mdot/{port}")
async def probe_mdot_path(port: str):
    """Path-style: `/probe/mdot/COM4`"""
    return await _probe_mdot_impl(port)


async def _probe_mdot_impl(port: str):
    """Diagnostic: try common baud rates with a short AT probe. POST /disconnect first if port is busy."""
    if lora.serial_connection is not None and lora.serial_connection.is_open:
        return {
            "error": "Serial port is in use — POST /disconnect first, then run probe again.",
            "port": port,
            "results": [],
            "suggested_baud": None,
        }
    results = await asyncio.to_thread(probe_mdot_uart, port)
    best = next((r for r in results if r.get("at_ok")), None)
    return {
        "port": port,
        "results": results,
        "suggested_baud": best["baud"] if best else None,
        "hint": (
            "If all rx_bytes are 0: wrong COM or TX/RX not reaching the mDot. "
            "If rx_hex is garbage at every baud: often wrong wiring or level mismatch. "
            "If at_ok true at one baud: use that baud in Connect."
        ),
    }


@app.post("/connect/{port}")
async def connect_port(port: str, baud: int = 9600, modem: str = "json"):
    """
    Connect to a COM port and attempt handshake with boat.
    Query: baud — must match device (mDot UDK often 115200).
    Query: modem — "json" (Pico / JSON bridge) or "mdot" (MultiTech AT on Micro UDK).
    """
    success = lora.connect(port, baudrate=baud, modem=modem)
    return {
        "success": success,
        "simulation": lora.simulation_mode,
        "boat_connected": lora.boat_connected,
        "modem": lora.modem_type,
    }


@app.post("/quickconnect/{port}")
async def quick_connect_port(port: str, baud: int = 9600, modem: str = "json"):
    """
    Quick connect - skips handshake, immediately switches to live mode.
    Query: modem — "json" or "mdot" (mDot: use placeholder telemetry until +EVT parsing).
    """
    success = lora.connect(port, baudrate=baud, modem=modem)
    if success:
        lora.simulation_mode = False
        lora.boat_connected = True
    return {
        "success": success,
        "simulation": lora.simulation_mode,
        "boat_connected": lora.boat_connected,
        "modem": lora.modem_type,
    }


@app.post("/handshake")
async def handshake_boat():
    """
    Perform handshake with boat - sends ping and waits for response.
    This is a blocking call that waits up to 3 seconds for boat response.
    """
    if not lora.serial_connection:
        return {
            "success": False,
            "error": "No serial connection",
            "simulation": True,
            "boat_connected": False
        }
    
    # Must await thread work — future.result() blocks the event loop and kills WebSocket pings
    timeout = 12.0 if lora.modem_type == "mdot" else 3.0
    boat_responded = await asyncio.to_thread(lora.ping_boat, timeout)
    
    return {
        "success": boat_responded,
        "simulation": lora.simulation_mode,
        "boat_connected": lora.boat_connected,
        "error": None if boat_responded else "Boat did not respond"
    }


@app.get("/status")
async def get_status():
    """Get current connection status."""
    return {
        "simulation": lora.simulation_mode,
        "boat_connected": lora.boat_connected,
        "serial_open": lora.serial_connection is not None and lora.serial_connection.is_open
    }


@app.post("/disconnect")
async def disconnect_port():
    """Disconnect from current port."""
    lora.disconnect()
    return {"success": True, "simulation": True, "boat_connected": False}


@app.post("/command")
async def send_command(command: ControlCommand):
    """Send control command to boat."""
    lora.send_command(command)
    return {"success": True}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket for real-time telemetry streaming."""
    await websocket.accept()
    lora.connected_clients.add(websocket)
    
    try:
        # Start sending telemetry
        while True:
            # Check for incoming commands
            try:
                data = await asyncio.wait_for(
                    websocket.receive_text(),
                    timeout=0.05
                )
                command = ControlCommand(**json.loads(data))
                lora.send_command(command)
            except asyncio.TimeoutError:
                pass
            except (json.JSONDecodeError, TypeError, ValueError):
                pass
            except Exception:
                pass

            try:
                telemetry = lora.get_telemetry()
                payload = json_safe_telemetry(telemetry)
            except Exception:
                telemetry = lora._generate_empty_telemetry()
                payload = json_safe_telemetry(telemetry)
            try:
                await websocket.send_json(payload)
            except Exception:
                break

            await asyncio.sleep(0.05)  # 20Hz update rate

    except WebSocketDisconnect:
        lora.connected_clients.remove(websocket)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

