#ifndef GYRO_SYSTEM_H
#define GYRO_SYSTEM_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>

class GyroSystem
{
public:
    GyroSystem();
    
    // Initialisierung des BNO055
    void begin();

    // Ruft man in jedem Loop-Zyklus auf, damit wir den aktuellen Winkel aktualisieren
    void update();

    // Liefert den zuletzt gelesenen Winkel in Grad (-180..+180)
    float getAngleDegrees() const;

    // Liefert den Winkel in Radiant (entspr. -PI..+PI)
    float getAngleRadians() const;

private:
    Adafruit_BNO055 bno;
    float gyroDegrees;
    float gyroRadiants;


};
extern GyroSystem gyros;  // globale Instanz

#endif
