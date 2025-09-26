#include "GyroSystem.h"

GyroSystem::GyroSystem() 
  : bno(55),      // Konstruktor-Parameter für den BNO (ID)
    gyroDegrees(0.0f),
    gyroRadiants(0.0f)
{
}

void GyroSystem::begin()
{
    // BNO055 starten
    if (!bno.begin())
    {
        Serial.println("Could not find a BNO055 sensor!");
        // Endlosschleife, falls Sensor nicht gefunden
        while (!bno.begin()) {
            Serial.println("Could not find a BNO055 sensor!");

            delay(500);
         }
    }

    // Optional: Additional config, offsets, calibration
    bno.setExtCrystalUse(true);

    bno.setMode(OPERATION_MODE_CONFIG);
      delay(25);
    
    bno.setMode(OPERATION_MODE_IMUPLUS);

          delay(25);



    

    

    Serial.println("BNO055 initialized.");
}

void GyroSystem::update()
{
    sensors_event_t event;
    bno.getEvent(&event);

    gyroDegrees = event.orientation.x;  // X-Achse => "Roll" oder "Heading", je nach Mode
    // Falls event.orientation.x in [0..360], wir wollen -180..+180
    if (gyroDegrees > 180.0f) {
        gyroDegrees -= 360.0f;
    }

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
