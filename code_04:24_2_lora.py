"""
=============================================================================
ANCHOR — BOAT LoRa BRIDGE PICO (RFM9x + I2C from sensor Pico)
=============================================================================
Put this file on the second Pico as code.py. Copy anchor_lora_compact.py to
the same CIRCUITPY drive.

Role:
  - Receives newline-terminated JSON telemetry from the sensor Pico on I2C
    (must include "lat", …) and transmits it over LoRa using pack_telemetry_11.
  - Receives LoRa uplink packets from ground station, decodes them, and
    forwards control commands to the sensor Pico over I2C.

I2C wiring (3 wires + common GND):
  Sensor Pico GP2 (SDA) <-> LoRa Pico GP2 (SDA)
  Sensor Pico GP3 (SCL) <-> LoRa Pico GP3 (SCL)
  GND <-> GND

LoRa RFM9x SPI (same as single-Pico layout):
  SCK  GP18   MOSI GP19   MISO GP16   CS GP17   RST GP22
  VCC 3.3V   GND GND

Air settings must match ground radio: 915 MHz, SF11, BW125k, CR4/5, CRC on.

USB: Windows often shows two COM ports for one Pico (console + data CDC). The
backend must reach the same interface the firmware reads, or use firmware that
drains both usb_cdc.console and usb_cdc.data (this sketch does).

Command forwarding (LoRa RX → I2C → sensor Pico):
  Shore Pico sends w/a/s/d/Enter as pack_control_11() or send_stop() over LoRa.
  This bridge decodes those packets and writes the resulting JSON command to
  the sensor Pico over I2C so process_command() can act on throttle/rudder/stop.

  Decoded command shapes forwarded to sensor:
    {"type":"control","throttle":<0..1>,"steering":<-1..1>}  ← w/a/s/d
    {"type":"stop"}                                           ← Enter key
    {"type":"ping"}                                           ← ping from shore
=============================================================================
"""

import board
import busio
import digitalio
import json
import supervisor
import sys
import time

import adafruit_rfm9x

try:
    from anchor_lora_compact import (
        LORA_BIN_LEN,
        TEL_MAGIC,
        CMD_MAGIC,
        pack_telemetry_11,
        pack_pong_11,
        unpack_cmd_11,
        is_ping_11,
    )
except ImportError:
    print("ERROR: anchor_lora_compact.py missing on CIRCUITPY")
    while True:
        time.sleep(1)

# --- I2C to sensor Pico target ---
I2C_SDA = board.GP2
I2C_SCL = board.GP3
I2C_ADDR = 0x42
I2C_FREQ = 400000
INTER_PICO_FRAME_MAGIC = 0xA5
INTER_PICO_FRAME_BYTES = 200

# --- LoRa SPI ---
LORA_SPI_SCK = board.GP18
LORA_SPI_MOSI = board.GP19
LORA_SPI_MISO = board.GP16
LORA_CS = board.GP17
LORA_RST = board.GP22
LORA_FREQ_MHZ = 915.0
LORA_SF = 11
LORA_BW = 125000
LORA_CR = 5
LORA_CRC = True
LORA_TX_POWER = 17

# Match boat code.py telemetry pacing when SF11 is heavy
LORA_MIN_TX_INTERVAL_S = 1.0
# SF11 @ BW125: ~15 B on-air is often ~0.35–0.7 s. Listen long enough for one packet, but do
# NOT block >1 s before I2C+TX each loop — that starves downlink and lets both ends sit in RX.
LORA_RX_POLL_TIMEOUT_S = 0.72

DEBUG_LORA_TX = True
# Chunk logs are noisy; set True only for I2C bring-up.
DEBUG_I2C_RX = False
DEBUG_LORA_RX = True
DEBUG_HEARTBEAT_S = 2.0
# Echo one JSON telemetry line per USB (GUI can use LoRa Pico COM if sensor USB is busy).
MIRROR_TELEMETRY_USB = True
USB_CMD_BUF_MAX = 512
# True: print each JSON object forwarded from USB host to I2C (throttle/rudder path).
DEBUG_USB_CMD = False


def main():
    i2c = busio.I2C(I2C_SCL, I2C_SDA, frequency=I2C_FREQ)

    spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)
    cs = digitalio.DigitalInOut(LORA_CS)
    cs.direction = digitalio.Direction.OUTPUT
    rst = digitalio.DigitalInOut(LORA_RST)
    rst.direction = digitalio.Direction.OUTPUT
    rfm = adafruit_rfm9x.RFM9x(spi, cs, rst, LORA_FREQ_MHZ)
    rfm.signal_bandwidth = LORA_BW
    rfm.spreading_factor = LORA_SF
    rfm.coding_rate = LORA_CR
    rfm.enable_crc = LORA_CRC
    rfm.tx_power = LORA_TX_POWER
    # FIX: removed redundant rfm.receive_timeout assignment — adafruit_rfm9x uses
    # the timeout= argument passed directly to receive(), not an object-level attribute.
    rfm.listen()

    print()
    print("ANCHOR boat LoRa bridge — I2C controller SDA GP2 / SCL GP3 addr 0x{:02X}".format(I2C_ADDR))
    print("I2C telemetry: framed 0x{:02X}+len+JSON (v2) or legacy JSON in read buffer (v1)".format(INTER_PICO_FRAME_MAGIC))
    print(
        "LoRa {:.1f} MHz SF{} BW{}k CR4/{} CRC={}  rx_poll={}s".format(
            LORA_FREQ_MHZ,
            LORA_SF,
            LORA_BW // 1000,
            LORA_CR,
            LORA_CRC,
            LORA_RX_POLL_TIMEOUT_S,
        )
    )
    print("LoRa RX → I2C command forwarding: ENABLED")

    last_tx_mono = 0.0
    last_hb = time.monotonic()
    i2c_bytes_total = 0
    i2c_frames_total = 0
    i2c_json_err_total = 0
    lora_rx_total = 0
    lora_rx_exc_total = 0
    lora_cmd_fwd_total = 0       # commands decoded from LoRa and forwarded to sensor
    usb_cmd_buf = ""
    i2c_read_err_total = 0
    i2c_write_ok_total = 0
    i2c_write_err_total = 0
    i2c_last_err = ""
    usb_bytes_in_total = 0
    usb_cmd_forward_total = 0
    i2c_target_ok = 0

    def drain_usb_to_sensor():
        """PC GUI/backend → USB on this Pico → forward control JSON to sensor over I2C."""
        nonlocal usb_cmd_buf, usb_bytes_in_total, usb_cmd_forward_total
        raw = bytearray()
        try:
            import usb_cdc

            streams = []
            cons = usb_cdc.console
            if cons is not None:
                streams.append(cons)
            data = getattr(usb_cdc, "data", None)
            if data is not None:
                streams.append(data)
            for stream in streams:
                try:
                    while stream.in_waiting:
                        nw = int(stream.in_waiting)
                        if nw <= 0:
                            break
                        chunk = stream.read(min(nw, 512))
                        if not chunk:
                            break
                        raw.extend(chunk)
                except Exception:
                    break
        except Exception:
            pass
        text = raw.decode("utf-8", "replace") if raw else ""
        if not text:
            try:
                while supervisor.runtime.serial_bytes_available:
                    c = sys.stdin.read(1)
                    if not c:
                        break
                    text += c
            except Exception:
                return
        if not text:
            return
        usb_bytes_in_total += len(text)
        for char in text:
            if char in "\n\r":
                if usb_cmd_buf:
                    line = usb_cmd_buf.strip()
                    usb_cmd_buf = ""
                    if line and line.startswith("{") and len(line) <= USB_CMD_BUF_MAX:
                        try:
                            obj = json.loads(line)
                            if isinstance(obj, dict) and obj.get("type"):
                                if DEBUG_USB_CMD:
                                    print("USB->I2C", obj.get("type"))
                                i2c_send_json(obj)
                                usb_cmd_forward_total += 1
                        except Exception:
                            pass
                else:
                    usb_cmd_buf = ""
            elif len(usb_cmd_buf) < USB_CMD_BUF_MAX:
                usb_cmd_buf += char
            else:
                usb_cmd_buf = ""

    def i2c_read_chunk(max_len=INTER_PICO_FRAME_BYTES):
        nonlocal i2c_read_err_total, i2c_last_err
        buf = bytearray(max_len)
        try:
            while not i2c.try_lock():
                pass
            try:
                i2c.readfrom_into(I2C_ADDR, buf)
            finally:
                i2c.unlock()
            return bytes(buf)
        except Exception as ex:
            i2c_read_err_total += 1
            i2c_last_err = str(ex)
            # Some firmwares behave better with shorter reads.
            try:
                small = bytearray(64)
                while not i2c.try_lock():
                    pass
                try:
                    i2c.readfrom_into(I2C_ADDR, small)
                finally:
                    i2c.unlock()
                return bytes(small)
            except Exception:
                return b""

    def parse_i2c_frame(chunk):
        if not chunk:
            return None
        if len(chunk) < 3:
            return None
        if chunk[0] != INTER_PICO_FRAME_MAGIC:
            return None
        n = int(chunk[1])
        if n <= 0 or (2 + n) > len(chunk):
            return None
        payload = bytes(chunk[2 : 2 + n])
        try:
            obj = json.loads(payload.decode("utf-8", "replace"))
        except Exception:
            return None
        return obj if isinstance(obj, dict) else None

    def parse_inter_pico_chunk(chunk):
        """
        Sensor may send v2 framed (0xA5,len,payload) or legacy UTF-8 JSON in the read buffer.
        Flash mismatch is common — accept both.
        """
        obj = parse_i2c_frame(chunk)
        if obj is not None:
            return obj
        raw = chunk.rstrip(b"\x00\xff")
        if not raw:
            return None
        if raw[0] == 0x7B:
            try:
                o = json.loads(raw.decode("utf-8", "replace"))
                return o if isinstance(o, dict) else None
            except Exception:
                pass
        i = raw.find(b"{")
        if i >= 0:
            try:
                o = json.loads(raw[i:].decode("utf-8", "replace"))
                return o if isinstance(o, dict) else None
            except Exception:
                pass
        return None

    def i2c_send_json(obj):
        """
        Write a JSON command to the sensor Pico over I2C.
        The sensor's drain_inter_pico_commands() reads these writes and passes
        them to process_command() — same path as USB control JSON from the GUI.
        """
        nonlocal i2c_write_ok_total, i2c_write_err_total, i2c_last_err
        if not isinstance(obj, dict):
            return
        try:
            payload = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
            while not i2c.try_lock():
                pass
            try:
                i2c.writeto(I2C_ADDR, payload[:240])
            finally:
                i2c.unlock()
            i2c_write_ok_total += 1
        except Exception as ex:
            i2c_write_err_total += 1
            i2c_last_err = str(ex)

    def log_lora_tx(kind, payload=None):
        """USB terminal debug for every outbound LoRa packet."""
        if not DEBUG_LORA_TX:
            return
        if kind == "telemetry" and isinstance(payload, dict):
            print(
                "LoRa TX telemetry: lat={:.6f} lon={:.6f} hdg={} spd={} bat={} sats={} obs={}".format(
                    float(payload.get("lat", 0.0)),
                    float(payload.get("lon", 0.0)),
                    payload.get("hdg", 0),
                    payload.get("spd", 0),
                    payload.get("bat", 100),
                    payload.get("sats", 0),
                    len(payload.get("obs", [])) if isinstance(payload.get("obs"), list) else 0,
                )
            )
            return
        if kind == "pong":
            print("LoRa TX pong (11B)")
            return
        print("LoRa TX", kind)

    # =========================================================================
    # handle_lora_rx — decode incoming LoRa bytes and forward to sensor Pico
    # =========================================================================
    def handle_lora_rx(raw):
        """
        Decode incoming LoRa bytes from the shore Pico and forward commands to
        the sensor Pico over I2C as newline-terminated JSON.

        The shore Pico (lora_receiver.py) sends:
          - w / s keys:  pack_control_11(throttle, 0.0, 0)
          - a / d keys:  pack_control_11(throttle, ±1.0, 0)  (impulse, steering resets immediately)
          - Enter key:   pack_control_11(0.0, 0.0, 0)  via send_stop()

        All of these unpack via unpack_cmd_11() to:
          {"type": "control", "throttle": <0..1>, "steering": <-1..1>}

        That JSON is written to the sensor Pico's I2C target address (0x42).
        The sensor's drain_inter_pico_commands() reads it and calls process_command(),
        which drives the ESC and rudder PWM outputs.

        ping packets (is_ping_11) are forwarded as {"type":"ping"} so the sensor
        can reply with a pong over USB / LoRa.
        """
        if not raw:
            return
        nonlocal lora_rx_total, lora_cmd_fwd_total
        lora_rx_total += 1
        if DEBUG_LORA_RX:
            print("LoRa RX {} bytes: {}".format(len(raw), " ".join("{:02X}".format(b) for b in raw)))

        cmd = None

        # --- 11-byte compact binary path (primary) ---
        if len(raw) == LORA_BIN_LEN:
            try:
                if is_ping_11(raw):
                    # Ping from shore: forward to sensor so it can pong back over USB.
                    cmd = {"type": "ping"}
                else:
                    # unpack_cmd_11 returns None for telemetry frames (TEL_MAGIC guard
                    # is inside anchor_lora_compact.py), so telemetry echoes are dropped.
                    decoded = unpack_cmd_11(raw)
                    if decoded is not None:
                        cmd = decoded
            except Exception as ex:
                if DEBUG_LORA_RX:
                    print("LoRa RX compact decode err:", ex)

        # --- UTF-8 JSON fallback (legacy / bench senders) ---
        if cmd is None:
            try:
                text = raw.decode("utf-8").strip()
                obj = json.loads(text)
                if isinstance(obj, dict) and obj.get("type"):
                    cmd = obj
            except Exception:
                pass

        if cmd is None:
            if DEBUG_LORA_RX:
                print("LoRa RX: unrecognised packet, ignored")
            return

        cmd_type = cmd.get("type", "?")
        if DEBUG_LORA_RX:
            if cmd_type == "control":
                print("LoRa RX cmd → control throttle={:.2f} steering={:+.2f} | forwarding to sensor".format(
                    float(cmd.get("throttle", 0.0)),
                    float(cmd.get("steering", 0.0)),
                ))
            else:
                print("LoRa RX cmd → {} | forwarding to sensor".format(cmd_type))

        # Write the decoded command to the sensor Pico via I2C.
        i2c_send_json(cmd)
        lora_cmd_fwd_total += 1

    while True:
        now = time.monotonic()

        drain_usb_to_sensor()

        # --- I2C poll + LoRa TX FIRST (ground must see telemetry; do not defer behind long RX) ---
        chunk = i2c_read_chunk()
        if chunk:
            i2c_bytes_total += len(chunk)
            obj = parse_inter_pico_chunk(chunk)
            if obj is None:
                i2c_json_err_total += 1
            else:
                i2c_frames_total += 1
                i2c_target_ok = 1
                if DEBUG_I2C_RX:
                    print("I2C RX frame:", str(obj)[:140])
                if "lat" in obj:
                    tnow = time.monotonic()
                    if (tnow - last_tx_mono) >= LORA_MIN_TX_INTERVAL_S:
                        try:
                            raw = pack_telemetry_11(
                                obj.get("lat"),
                                obj.get("lon"),
                                obj.get("hdg", 0),
                                obj.get("spd", 0),
                                obj.get("bat", 100),
                                obj.get("sats", 0),
                                obj.get("obs", []),
                            )
                            rfm.send(raw, keep_listening=True)
                            log_lora_tx("telemetry", obj)
                            if MIRROR_TELEMETRY_USB:
                                try:
                                    print(json.dumps(obj, separators=(",", ":")))
                                except Exception:
                                    pass
                            last_tx_mono = tnow
                        except Exception as ex:
                            print("LoRa TX err:", ex)
                            # FIX: recover RX mode if TX fails — rfm can get stuck after
                            # a send() exception and silently stop receiving.
                            try:
                                rfm.listen()
                            except Exception:
                                pass
                elif obj.get("type") == "pong":
                    try:
                        rfm.send(pack_pong_11(), keep_listening=True)
                        log_lora_tx("pong")
                    except Exception as ex:
                        print("LoRa pong err:", ex)
                        # FIX: same RX recovery for pong TX failures.
                        try:
                            rfm.listen()
                        except Exception:
                            pass

        drain_usb_to_sensor()

        # --- LoRa RX: poll for incoming packets (commands from shore Pico) ---
        try:
            pkt = rfm.receive(timeout=LORA_RX_POLL_TIMEOUT_S)
        except Exception as ex:
            lora_rx_exc_total += 1
            if DEBUG_LORA_RX or (lora_rx_exc_total <= 3):
                print("LoRa receive() error:", ex)
            pkt = None
        if pkt:
            handle_lora_rx(bytes(pkt))

        drain_usb_to_sensor()

        if DEBUG_HEARTBEAT_S > 0 and (now - last_hb) >= DEBUG_HEARTBEAT_S:
            last_hb = now
            crc_n = getattr(rfm, "crc_error_count", -1)
            rssi_v = getattr(rfm, "last_rssi", None)
            rssi_s = "{:.0f}".format(rssi_v) if isinstance(rssi_v, (int, float)) else "?"
            print(
                "HB i2c_bytes={} i2c_frames={} json_err={} lora_rx={} lora_cmd_fwd={} lora_exc={} crc_err={} last_rssi={} i2c_ok={} rd_err={} wr_ok={} wr_err={} usb_in={} usb_fwd={}".format(
                    i2c_bytes_total,
                    i2c_frames_total,
                    i2c_json_err_total,
                    lora_rx_total,
                    lora_cmd_fwd_total,
                    lora_rx_exc_total,
                    crc_n,
                    rssi_s,
                    int(i2c_target_ok),
                    i2c_read_err_total,
                    i2c_write_ok_total,
                    i2c_write_err_total,
                    usb_bytes_in_total,
                    usb_cmd_forward_total,
                )
            )
            if i2c_last_err:
                print("I2C last err:", i2c_last_err)

        time.sleep(0.002)


try:
    main()
except KeyboardInterrupt:
    print("Stopped.")
