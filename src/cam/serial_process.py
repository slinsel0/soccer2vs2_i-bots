import serial
import struct
import time
import zlib
from cobs import cobs # Ensure 'cobs' is installed: pip install cobs

def run_serial(config, stop_event, result_queue):
    """
    Reads analysis results and sends them via USB/Serial to the microcontroller.
    Uses COBS encoding and CRC32 for robustness.
    """
    port = config['serial']['port']
    baud = config['serial']['baud']
    
    try:
        ser = serial.Serial(port, baud, timeout=0)
    except Exception as e:
        print(f"[SER] Error opening serial: {e}")
        return

    # Frame Structure: <BBIfff (ID, Seq, Timestamp, Dist, Angle, Valid)
    # Using float for compatibility with your existing struct logic
    FMT_BODY = "<BBIfff"
    MSG_ID = 1
    seq = 0

    print("[SER] Serial Comm Started")

    while not stop_event.is_set():
        try:
            # Get latest data
            data = result_queue.get(timeout=1.0)
        except:
            continue
        
        if data.get('found', False):
            t_us = data['t_us']
            dist = float(data['dist'])
            angle = float(data['angle'])
            valid = 1.0
        else:
            t_us = time.perf_counter_ns() // 1000
            dist = 0.0
            angle = 0.0
            valid = 0.0

        # Pack
        body = struct.pack(FMT_BODY, MSG_ID, seq & 0xFF, t_us & 0xFFFFFFFF, dist, angle, valid)
        
        # CRC
        crc = zlib.crc32(body) & 0xFFFFFFFF
        frame = body + struct.pack("<I", crc)
        
        # Encode & Send
        encoded = cobs.encode(frame) + b'\x00'
        ser.write(encoded)
        
        seq += 1

    ser.close()
    print("[SER] Stopped.")