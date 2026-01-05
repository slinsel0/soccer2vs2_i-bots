#include "GyroSystem.h"
#include "Arduino.h"

// Konstruktor: Initialisiert Variablen
GyroSystem::GyroSystem() 
  : gyroDegrees(0.0f),
    gyroRadiants(0.0f),
    angleOffset(0.0f)
{
}

void GyroSystem::begin()
{
    // BNO08x mit Standard-I2C-Adresse starten
    if (!bno08x.begin_I2C())
    {
        Serial.println("Fehler: BNO08x nicht gefunden!");
        while (true) {
            if (bno08x.begin_I2C()) break;
            Serial.println("Suche BNO08x...");
            delay(1000);
         }
    }

    Serial.println("BNO08x gefunden!");

    // Game Rotation Vector aktivieren (Kein Magnetometer, ähnlich IMUPLUS)
    // 10ms Report-Rate (100Hz)
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) { 
        Serial.println("Konnte Game Rotation Vector nicht aktivieren");
    }
}

void GyroSystem::update()
{
    sh2_SensorValue_t sensorValue;
    
    // Prüfen ob neue Daten da sind
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR: {
                // Quaternion-Werte
                float qi = sensorValue.un.gameRotationVector.i;
                float qj = sensorValue.un.gameRotationVector.j;
                float qk = sensorValue.un.gameRotationVector.k;
                float qr = sensorValue.un.gameRotationVector.real;
                
                // Umrechnung Quaternion -> Euler (Yaw / Heading)
                // Standard-Methode:
                // yaw = atan2(2*(r*k + i*j), 1 - 2*(j*j + k*k))
                // (Abhängig von Sensor-Ausrichtung. Hier Standardannahme)
                
                float siny_cosp = 2.0f * (qr * qk + qi * qj);
                float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
                float yaw = atan2(siny_cosp, cosy_cosp);
                
                // Radian -> Grad
                float rawDeg = yaw * 180.0f / PI;

                // BNO08x liefert -180..+180 (positiv CCW normalerweise)
                // BNO055 (alt) lieferte 0..360 CW (Clockwise). 
                // Um Kompatibilität zu wahren (rechts positiv), müssen wir evtl. invertieren.
                // Wenn wir davon ausgehen, dass rawDeg hier CCW ist (Standard Mathe):
                // rechts drehen -> Winkel wird negativ
                // links drehen -> Winkel wird positiv
                
                // Der alte Code:
                // raw = 0..360.
                // if (raw > 180) raw -= 360.
                // 10 Grad rechts -> 10.
                // 10 Grad links -> 350 -> -10.
                // Also WAR rechts positiv, links negativ.
                
                // Wenn atan2 CCW ist:
                // 10 Grad rechts -> -10.
                // 10 Grad links -> +10.
                // -> Wir müssen invertieren, damit rechts positiv ist.
                
                rawDeg = -rawDeg; 

                gyroDegrees = rawDeg;
                gyroRadiants = gyroDegrees * PI / 180.0f;
                break;
            }
        }
    }
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