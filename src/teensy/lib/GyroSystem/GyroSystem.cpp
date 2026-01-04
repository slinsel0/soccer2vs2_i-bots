#include "GyroSystem.h"
#include "Arduino.h"

// Konstruktor: Initialisiert BNO-ID und Variablen
GyroSystem::GyroSystem() 
  : bno(55, 0x28), // ID 55, Adresse 0x28 (Standard) oder 0x29 prüfen!
    gyroDegrees(0.0f),
    gyroRadiants(0.0f),
    angleOffset(0.0f)
{
}

void GyroSystem::begin()
{
    // WICHTIG: Starten im IMUPLUS Modus!
    // OPERATION_MODE_NDOF    = Standard (nutzt Magnetometer -> schlecht bei Motoren)
    // OPERATION_MODE_IMUPLUS = Nur Gyro + Accel (Magnetometer deaktiviert -> stabil)
    
    if (!bno.begin())
    {
        Serial.println("Fehler: BNO055 nicht gefunden!");
        // Endlosschleife mit Blink-Versuch, falls Sensor fehlt
        while (true) {
            if (bno.begin(OPERATION_MODE_IMUPLUS)) break;
            Serial.println("Suche BNO055...");
            delay(1000);
         }
    }

    // Externen Quarz nutzen (besser für Präzision)
    bno.setExtCrystalUse(true);
    
    Serial.println("BNO055 gestartet im IMUPLUS-Modus (Kein Magnetometer).");
}

void GyroSystem::update()
{
    sensors_event_t event;
    bno.getEvent(&event);

    // Rohwert vom Sensor (Euler-Winkel X ist das "Heading")
    float raw = event.orientation.x; 

    // Offset anwenden (für Drift-Korrektur durch LiDAR)
    

    // Normalisierung auf -180..+180 Grad (damit 0 vorne ist und links/rechts +/-)
    // Der BNO gibt 0..360 aus.
    
    // Erstmal auf 0..360 normalisieren (durch Offset kann es <0 oder >360 sein)
  

    // Jetzt Umrechnung in -180..+180
    if (raw > 180.0f) {
        raw -= 360.0f;
    }

    gyroDegrees = raw;
    gyroRadiants = gyroDegrees * PI / 180.0f;
}

float GyroSystem::getAngleDegrees() const 
{
    return gyroDegrees;
}

float GyroSystem::getAngleRadians() const
{
    return gyroRadiants;
}

// Setzt einen harten neuen Nullpunkt (z.B. per Knopfdruck)
void GyroSystem::setOffset(float deg) {
    // Wenn wir sagen "Hier ist jetzt 0", müssen wir den Offset so setzen,
    // dass (raw + offset) = 0 ergibt. Also offset = -raw.
    // Hier speicherst du aber einen additiven Offset zur Korrektur.
    // Einfacher Setter:
    angleOffset = deg;
}

// Integriert kleine Korrekturen (vom LiDAR)
void GyroSystem::adjustOffset(float deltaDeg) {
    angleOffset += deltaDeg;
    
    // Offset sauber halten
    while (angleOffset > 180.0f) angleOffset -= 360.0f;
    while (angleOffset <= -180.0f) angleOffset += 360.0f;
}