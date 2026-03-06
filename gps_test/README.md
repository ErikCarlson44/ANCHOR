# GPS Test - GY-GPSV3 NEO-M8N

Test script for GPS module with NMEA parsing.

## Wiring

| GPS Module | Pico Pin |
|------------|----------|
| TX | GP5 (Pico RX) |
| RX | GP4 (Pico TX) |
| VCC | 3.3V |
| GND | GND |

**Note:** GPS TX connects to Pico RX (GP5), GPS RX connects to Pico TX (GP4).

Using GP4/GP5 to avoid conflict with radar (which uses GP0/GP1).

## Usage

1. Copy `gps_test.py` to Pico as `code.py`
2. Open Thonny serial console
3. Wait for GPS fix (30-60 seconds outdoors)

## Output

**With fix:**
```
FIX | Lat: 32.787200 | Lon: -117.235000 | Spd: 0.0 kts | Hdg: 45.0° | Sats: 8
```

**Without fix:**
```
NO FIX | Sats: 3 | Searching...
```

## NMEA Sentences Parsed

| Sentence | Data |
|----------|------|
| $GPGGA / $GNGGA | Lat, Lon, Altitude, Fix Quality, Satellites |
| $GPRMC / $GNRMC | Lat, Lon, Speed (knots), Heading |

## Configuration

Default settings in code:
```python
GPS_TX_PIN = board.GP4   # Pico TX -> GPS RX
GPS_RX_PIN = board.GP5   # Pico RX <- GPS TX
GPS_BAUDRATE = 9600      # NEO-M8N default
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No data at all | Check TX/RX wiring (swap if needed) |
| Garbage characters | Wrong baud rate |
| "NO FIX" forever | Go outdoors, clear sky view needed |
| Fix takes long time | Cold start can take 1-5 minutes |

## GPS Class API

```python
from gps_test import GPSReader

gps = GPSReader()

while True:
    gps.update()  # Call frequently
    data = gps.get_data()
    
    # data = {
    #     "lat": 32.7872,      # Latitude (decimal degrees)
    #     "lon": -117.235,     # Longitude (decimal degrees)
    #     "alt": 15.0,         # Altitude (meters)
    #     "hdg": 45.0,         # Heading (degrees)
    #     "spd": 5.2,          # Speed (knots)
    #     "sats": 8,           # Satellites in use
    #     "fix": True          # Has valid fix
    # }
```
