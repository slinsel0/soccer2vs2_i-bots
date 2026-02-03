#!/bin/bash
# Gehe sicher, dass wir im richtigen Ordner sind
cd src/teensy

echo "Baue Image (falls nötig)..."
# Wir bauen das Image nur neu, wenn sich das Dockerfile oder die platformio.ini ändert
# Das > /dev/null unterdrückt die Ausgabe, damit es übersichtlich bleibt
docker build -t bots-teensy . 

echo "Starte Upload..."
# WICHTIG: -v /dev:/dev ist robuster als nur /dev/bus/usb
# --privileged ist zwingend nötig für den Hardware-Zugriff beim Reset
docker run -it --rm \
  --privileged \
  -v /dev:/dev \
  -v "$(pwd)":/app \
  bots-teensy \
  pio run --target upload
