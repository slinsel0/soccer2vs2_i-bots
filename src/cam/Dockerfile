FROM python:3.11-bookworm

# 1. RPi-Repository hinzufügen (WICHTIG!)
# Das Standard-Python-Image kennt "python3-libcamera" nicht. 
# Wir fügen die Raspberry Pi Quellen hinzu.
RUN apt-get update && apt-get install -y wget gpg \
    && wget -qO - https://archive.raspberrypi.com/debian/raspberrypi.gpg.key | gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] http://archive.raspberrypi.com/debian/ bookworm main" | tee /etc/apt/sources.list.d/raspberrypi.list \
    && apt-get update

# 2. System-Abhängigkeiten installieren
# Jetzt findet er auch die speziellen Pi-Pakete
RUN apt-get install -y \
    libcamera-dev \
    libcamera-tools \
    python3-libcamera \
    python3-kms++ \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# WICHTIG: Damit unser Python die per apt installierten Pakete (picamera2) findet
ENV PYTHONPATH="/usr/lib/python3/dist-packages"

# 3. Arbeitsverzeichnis
WORKDIR /app

# 4. Python Pakete installieren
# Wir nutzen --break-system-packages, da wir im Container sind
COPY requirements.txt .
RUN pip install --no-cache-dir --break-system-packages -r requirements.txt

# 5. Startbefehl
CMD ["python3", "main.py"]
