# WICHTIG: Wir wechseln von 'python:3.11' zu 'debian:bookworm'.
# Grund: 'python:3.11' hat eine eigene Python-Installation, die die apt-Pakete
# (wie python3-picamera2 oder python3-opencv) nicht sieht.
# Mit 'debian:bookworm' nutzen wir das System-Python für alles.
FROM debian:bookworm

# Umgebungsvariablen
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /app

# 1. Raspberry Pi Repository hinzufügen
RUN apt-get update && apt-get install -y wget gpg && \
    wget -qO - https://archive.raspberrypi.org/debian/raspberrypi.gpg.key | gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] http://archive.raspberrypi.org/debian/ bookworm main" | tee /etc/apt/sources.list.d/raspberrypi.list

# 2. Systempakete & Python installieren
# Wir installieren jetzt python3 direkt via apt, damit es kompatibel mit den Libs ist.
# NEU: xauth und x11-apps für GUI/Window-Forwarding hinzugefügt.
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-picamera2 \
    python3-libcamera \
    python3-pyqt5 \
    libcamera-apps \
    libcamera-tools \
    v4l-utils \
    libgl1-mesa-glx \
    python3-opencv \
    python3-numpy \
    xauth \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# 3. Requirements verarbeiten
COPY requirements.txt .

# Wir entfernen Pakete, die wir schon via APT installiert haben, um Konflikte zu vermeiden.
RUN sed -i '/picamera2/d' requirements.txt && \
    sed -i '/opencv-python/d' requirements.txt && \
    sed -i '/opencv-contrib-python/d' requirements.txt && \
    sed -i '/numpy/d' requirements.txt

# --break-system-packages erlaubt pip, Pakete ins System-Python zu installieren.
# In Docker ist das sicher, da der Container isoliert ist.
RUN pip3 install -r requirements.txt --break-system-packages

# 4. Code kopieren
COPY . .

# Startbefehl: Explizit python3 nutzen
CMD ["python3", "main.py"]
