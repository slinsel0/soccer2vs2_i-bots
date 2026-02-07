# Wir nutzen ein schlankes Python Image als Basis
FROM python:3.11-slim-bookworm

# 1. Systempakete installieren
RUN apt-get update && apt-get install -y \
    build-essential \
    libusb-dev \
    libusb-1.0-0-dev \
    udev \
    git \
    wget \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*


# 2. PlatformIO Core, matplotlib und pyserial installieren
RUN pip install -U platformio matplotlib pyserial

# 3. Teensy Loader CLI von Source kompilieren
WORKDIR /tmp
RUN git clone https://github.com/PaulStoffregen/teensy_loader_cli.git \
    && cd teensy_loader_cli \
    && make \
    && mv teensy_loader_cli /usr/local/bin/ \
    && cd .. \
    && rm -rf teensy_loader_cli

# 4. DEPENDENCY CACHING (Der neue Teil!)
WORKDIR /app
# Kopiere nur die Konfiguration
COPY platformio.ini /app/platformio.ini
# Erstelle einen Dummy-Code, damit wir einen Build anstoßen können
RUN mkdir -p src && echo "void setup(){} void loop(){}" > src/main.cpp
# Führe einen Build aus -> Das lädt ALLE Libraries und Toolchains herunter und speichert sie im Image
RUN pio run || true
# Bereinige den Dummy-Code wieder (damit wir später deinen echten Code nutzen)
RUN rm src/main.cpp
RUN pio run --target clean

# 5. Standardbefehl
CMD ["pio", "--help"]