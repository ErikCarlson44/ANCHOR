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
from typing import Optional, List, Set

import serial
import serial.tools.list_ports
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel


# ============== Data Models ==============

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
        
    def list_ports(self) -> List[str]:
        """List available COM ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to LORA module (serial port only, not boat)."""
        try:
            if self.serial_connection:
                self.serial_connection.close()
            self.serial_connection = serial.Serial(port, baudrate, timeout=1)
            # Don't set simulation_mode=False yet - wait for boat handshake
            return True
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            return False
    
    def ping_boat(self, timeout: float = 3.0) -> bool:
        """
        Send ping to boat and wait for response.
        Returns True if boat responds, False otherwise.
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            return False
        
        try:
            # Clear any pending data
            self.serial_connection.reset_input_buffer()
            
            # Send ping message
            ping_msg = json.dumps({"type": "ping"}) + '\n'
            self.serial_connection.write(ping_msg.encode())
            print(f"Sent ping to boat...")
            
            # Wait for response with timeout
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if self.serial_connection.in_waiting > 0:
                    try:
                        line = self.serial_connection.readline().decode().strip()
                        if line:
                            data = json.loads(line)
                            # Check for pong response or telemetry (either means boat is alive)
                            if data.get('type') == 'pong' or 'lat' in data:
                                print(f"Boat responded!")
                                self.boat_connected = True
                                self.simulation_mode = False
                                return True
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        pass  # Keep waiting
                time.sleep(0.05)
            
            print(f"Boat did not respond within {timeout}s")
            return False
            
        except serial.SerialException as e:
            print(f"Ping failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from LORA module."""
        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None
        self.simulation_mode = True
        self.boat_connected = False
    
    def send_command(self, command: ControlCommand):
        """Send control command to boat."""
        if command.type == "control":
            self._sim_throttle = command.throttle or 0.0
            self._sim_steering = command.steering or 0.0
        elif command.type == "stop":
            self._sim_throttle = 0.0
            self._sim_steering = 0.0
        
        if self.serial_connection and self.serial_connection.is_open:
            cmd_str = json.dumps(command.dict()) + '\n'
            self.serial_connection.write(cmd_str.encode())
    
    def get_telemetry(self) -> BoatTelemetry:
        """Get current telemetry (from serial or simulation)."""
        if self.simulation_mode:
            return self._generate_simulation()
        else:
            return self._read_serial()
    
    def _read_serial(self) -> Optional[BoatTelemetry]:
        """Read telemetry from serial port."""
        if not self.serial_connection or not self.serial_connection.is_open:
            return self._generate_simulation()
        
        try:
            if self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode().strip()
                data = json.loads(line)
                return BoatTelemetry(
                    latitude=data.get('lat', 0),
                    longitude=data.get('lon', 0),
                    heading=data.get('hdg', 0),
                    speed=data.get('spd', 0),
                    battery=data.get('bat', 100),
                    obstacles=[
                        {'distance': o[0], 'angle': o[1], 'size': o[2]}
                        for o in data.get('obs', [])
                    ],
                    timestamp=time.time()
                )
        except (json.JSONDecodeError, serial.SerialException):
            pass
        
        return self._generate_simulation()
    
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
        
        # Generate random obstacles
        obstacles = []
        for _ in range(random.randint(0, 4)):
            obstacles.append({
                'distance': random.uniform(5, 50),
                'angle': random.uniform(-90, 90),
                'size': random.uniform(0.5, 3)
            })
        
        return BoatTelemetry(
            latitude=self._sim_lat,
            longitude=self._sim_lon,
            heading=self._sim_heading,
            speed=abs(self._sim_speed),
            battery=self._sim_battery,
            obstacles=obstacles,
            timestamp=time.time()
        )


# ============== FastAPI App ==============

lora = LoraHandler()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("=== RC Boat Control Backend Starting ===")
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


@app.post("/connect/{port}")
async def connect_port(port: str):
    """
    Connect to a COM port and attempt handshake with boat.
    Returns immediately after opening serial port.
    Use /handshake endpoint to perform the actual boat connection.
    """
    success = lora.connect(port)
    return {
        "success": success, 
        "simulation": lora.simulation_mode,
        "boat_connected": lora.boat_connected
    }


@app.post("/quickconnect/{port}")
async def quick_connect_port(port: str):
    """
    Quick connect - skips handshake, immediately switches to live mode.
    Use this for testing when boat handshake isn't working.
    """
    success = lora.connect(port)
    if success:
        lora.simulation_mode = False
        lora.boat_connected = True
    return {
        "success": success,
        "simulation": lora.simulation_mode,
        "boat_connected": lora.boat_connected
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
    
    # Run ping in thread to not block event loop
    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor() as executor:
        future = executor.submit(lora.ping_boat, 3.0)
        boat_responded = future.result()
    
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
            
            # Send telemetry
            telemetry = lora.get_telemetry()
            await websocket.send_json(telemetry.dict())
            
            await asyncio.sleep(0.05)  # 20Hz update rate
            
    except WebSocketDisconnect:
        lora.connected_clients.remove(websocket)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

