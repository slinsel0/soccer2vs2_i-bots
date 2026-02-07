#!/bin/bash
# Gehe sicher, dass wir im richtigen Ordner sind (analog zu flash_teensy.sh)
cd src/teensy 2>/dev/null || true

echo "Baue Image (falls nötig)..."
docker build -t bots-teensy . > /dev/null

echo "Starte Serial Monitor..."

# Versuche den ersten ttyACM Port zu finden
TEENSY_PORT=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)

if [ -z "$TEENSY_PORT" ]; then
    echo "Kein Teensy gefunden (/dev/ttyACM*). Versuche trotzdem zu starten..."
    DEVICE_ARG=""
else
    echo "Teensy gefunden an $TEENSY_PORT"
    DEVICE_ARG="--device=$TEENSY_PORT"
fi

# Starte den Serial Monitor innerhalb des Containers
# Wir nutzen denselben Container wie beim Flashen
docker run -it --rm \
  --privileged \
  -v "$(pwd)":/app \
  $DEVICE_ARG \
  bots-teensy \
  pio device monitor --baud 2000000
