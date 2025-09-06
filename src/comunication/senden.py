# file: pi_tx.py
# Start:  python3 pi_tx.py  (achte auf den richtigen /dev/ttyACM*)
import time, struct, zlib, serial
from cobs import cobs

PORT = "/dev/ttyACM0"     # ggf. /dev/ttyACM1
BAUD = 2_000_000          # für USB-CDC egal, schadet aber nicht
ser  = serial.Serial(PORT, BAUD, timeout=0)

# Frame-Layout (Little-Endian):
# <BBIfffI  -> msg_id:uint8, seq:uint8, t_us:uint32, vx:float32, vy:float32, omega:float32, crc32:uint32
FMT_BODY = "<BBIfff"
FMT_FULL = "<BBIfffI"
MSG_ID_VELOCITY = 1

seq = 0

def make_frame(vx: float, vy: float, omega: float) -> bytes:
    global seq
    t_us = time.perf_counter_ns() // 1000  # Mikrosekunden (lokal)
    body = struct.pack(FMT_BODY, MSG_ID_VELOCITY, seq & 0xFF, t_us & 0xFFFFFFFF,
                       float(vx), float(vy), float(omega))
    crc  = zlib.crc32(body) & 0xFFFFFFFF   # IEEE CRC-32 (zlib)
    frame = body + struct.pack("<I", crc)  # CRC hinten anhängen
    seq = (seq + 1) & 0xFF
    # COBS + 0x00-Delimiter
    return cobs.encode(frame) + b"\x00"

def send_vector(vx, vy, omega):
    ser.write(make_frame(vx, vy, omega))

def poll_rx_once():
    """Optional: liest eingehende COBS-Frames (z.B. Acks/Telemetrie)."""
    static_buf = getattr(poll_rx_once, "_buf", bytearray())
    data = ser.read(1024)
    if not data:
        return
    for ch in data:
        if ch == 0:  # Frame fertig
            if static_buf:
                try:
                    decoded = cobs.decode(bytes(static_buf))
                    if len(decoded) >= 22:
                        body, crc_bytes = decoded[:-4], decoded[-4:]
                        rx_crc = struct.unpack("<I", crc_bytes)[0]
                        if (zlib.crc32(body) & 0xFFFFFFFF) != rx_crc:
                            # CRC-Fehler -> verwerfen
                            pass
                        else:
                            # Beispiel: falls ein Vector-Frame zurückkommt
                            msg_id, s, t_us, vx, vy, omg = struct.unpack(FMT_BODY, body)
                            print(f"RX msg={msg_id} seq={s} t_us={t_us} vx={vx:.3f} vy={vy:.3f} w={omg:.3f}")
                except Exception:
                    pass
            static_buf.clear()
        else:
            static_buf.append(ch)
    poll_rx_once._buf = static_buf

if __name__ == "__main__":
    print(f"Sende Vektoren an {PORT} … (Strg+C zum Beenden)")
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            # Beispielwerte: 200 Hz
            send_vector(0.8, 0.0, 0.4)
            poll_rx_once()
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
