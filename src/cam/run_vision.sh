#!/bin/bash
cd src/cam

echo "Stelle sicher, dass Teensy da ist..."
if [ ! -e /dev/ttyACM0 ]; then
    echo "WARNUNG: Teensy (/dev/ttyACM0) nicht gefunden!"
fi

echo "Starte Vision System..."
docker build -t bots-vision . > /dev/null

docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $HOME/.Xauthority:/root/.Xauthority \
    --device /dev/ttyACM0 \
    -v /run/udev:/run/udev \
    -v $(pwd):/app \
    bots-vision
