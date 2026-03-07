#ifndef GYRO_SYSTEM_H
#define GYRO_SYSTEM_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <Arduino.h>

class GyroSystem
{
public:
    GyroSystem();
    
    void begin();
    void update();

    // Liefert den korrigierten Winkel
    float getAngleDegrees() const;
    float getAngleRadians() const;

    // --- NEU: Drift-Korrektur ---
    // Setzt einen harten Offset (z.B. beim Start oder Reset)
    void setOffset(float deg);
    
    // Addiert eine kleine Korrektur (für kontinuierlichen Drift-Fix durch LiDAR)
    void adjustOffset(float deltaDeg);

    float getAccelX() const;
    float getAccelY() const;

private:
   Adafruit_BNO08x  bno08x;
    float gyroDegrees;
    float gyroRadiants;
    
    float angleOffset; // Der Korrekturwert
    float accelX;
    float accelY;
};
extern GyroSystem gyro;

#endif