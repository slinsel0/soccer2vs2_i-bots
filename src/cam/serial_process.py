import serial
import struct
import time
import zlib
from cobs import cobs 

def run_serial(config, stop_event, result_queue):
    """
    Reads analysis results and sends them via USB/Serial to the microcontroller.
    ROBUST VERSION: Does not crash if serial is missing. Tries to reconnect.
    """
    port = config['serial']['port']
    baud = config['serial']['baud']
    
    ser = None
    
    # Frame Structure: <BBIfff (ID, Seq, Timestamp, Dist, Angle, Valid)
    FMT_BODY = "<BBIfff"
    MSG_ID = 1
    seq = 0

    print("[SER] Serial Process Started (Robust Mode)")

    while not stop_event.is_set():
        # --- 1. Verbindung herstellen (Falls nicht verbunden) ---
        if ser is None:
            try:
                ser = serial.Serial(port, baud, timeout=0)
                print(f"[SER] Verbunden mit {port}")
            except Exception as e:
                # Kein Port da? Kein Problem. Warte kurz und versuche es erneut.
                print(f"[SER] Warte auf Serial Port {port}... ({e})")
                time.sleep(2.0) 
                continue # Springe zum Anfang der Schleife

        # --- 2. Daten senden ---
        try:
            # Versuche Daten aus der Queue zu holen
            try:
                data = result_queue.get(timeout=0.01) # Nicht blockieren
            except:
                # Keine neuen Daten? Sende nichts (oder Heartbeat, falls gewünscht)
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

            # Packen
            body = struct.pack(FMT_BODY, MSG_ID, seq & 0xFF, t_us & 0xFFFFFFFF, dist, angle, valid)
            
            # CRC & Encoding
            crc = zlib.crc32(body) & 0xFFFFFFFF
            frame = body + struct.pack("<I", crc)
            encoded = cobs.encode(frame) + b'\x00'
            
            # Schreiben
            ser.write(encoded)
            seq += 1

        except (OSError, serial.SerialException) as e:
            print(f"[SER] Verbindung verloren: {e}")
            try:
                ser.close()
            except:
                pass
            ser = None # Setze auf None, damit oben neu verbunden wird
            
        except Exception as e:
            print(f"[SER] Unerwarteter Fehler: {e}")

    # Aufräumen beim Beenden
    if ser:
        ser.close()
    print("[SER] Stopped.")