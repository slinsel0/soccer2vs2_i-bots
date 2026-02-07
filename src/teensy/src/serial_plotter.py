import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import time
import threading

# Konfiguration
BAUDRATE = 2000000
MAX_POINTS = 100

def get_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("Keine seriellen Ports gefunden.")
        return None
    # Nimm den ersten gefundenen Port (meistens /dev/ttyACM0 oder ähnlich im Container)
    # Im Docker Container wird der durchgereichte Port oft als /dev/ttyACM0 sichtbar sein
    for p in ports:
        if "Teensy" in p.description or "ttyACM" in p.device:
             return p.device
    return ports[0].device # Fallback

def read_serial(ser, data_list):
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    # Erwarte Format: "wert1, wert2, ..." oder einfach nur eine Zahl
                    # Hier einfachheitshalber: Plotte den ersten numerischen Wert
                    parts = line.split(',')
                    val = float(parts[0])
                    data_list.append(val)
                    if len(data_list) > MAX_POINTS:
                        data_list.pop(0)
                except ValueError:
                    pass # Kein numerischer Wert
        except Exception as e:
            print(f"Fehler beim Lesen: {e}")
            break

def main():
    port = get_serial_port()
    if len(sys.argv) > 1:
        port = sys.argv[1]

    if not port:
        print("Kein Port angegeben oder gefunden.")
        sys.exit(1)

    print(f"Verbinde mit {port} bei {BAUDRATE} Baud...")

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
    except serial.SerialException as e:
        print(f"Konnte Port nicht öffnen: {e}")
        sys.exit(1)

    data = []

    # Starte Thread zum Lesen der Daten
    t = threading.Thread(target=read_serial, args=(ser, data), daemon=True)
    t.start()

    # Plot Setup
    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2)
    ax.set_ylim(-10, 100) # Automatische Skalierung wäre besser, aber fixiert für Stabilität erst mal
    ax.set_xlim(0, MAX_POINTS)
    ax.grid()

    def update(frame):
        line.set_data(range(len(data)), data)
        if data:
             ax.set_ylim(min(data) - 10, max(data) + 10)
        return line,

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.show()

if __name__ == "__main__":
    main()
