#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <math.h>

// ------------------- MotorPins -------------------
static const uint8_t motorHRpin[] = {3, 4, 15};   // HR: CCW1, CW1, PWM
static const uint8_t motorVRpin[] = {5, 6, 19};   // VR: CCW1, CW1, PWM
static const uint8_t motorVLpin[] = {10, 7, 14};  // VL: CCW1, CW1, PWM
static const uint8_t motorHLpin[] = {12, 11, 18}; // HL: CCW1, CW1, PWM

// ------------------ MotorVersatz (Winkel in Radiant) -----------------
#define motorVRversatz (PI * 50.0f  / 180.0f)
#define motorHRversatz (PI * 130.0f / 180.0f)
#define motorHLversatz (PI * 230.0f / 180.0f)
#define motorVLversatz (PI * 310.0f / 180.0f)

// ----------------- MinMaxMotor -------------------
#define maxSpeed     120  // maximaler Geschwindigkeitswert
#define minSpeed     27    // minimaler Speed
#define maxRotation  80    // maximale Rotationsgeschwindigkeit




// ----------------- Radpositionen -----------------
// Angenommene Werte (in cm oder passenden Einheiten):
static const float motorPosX[4] = {
    5.95f,    // VR (50°)
    5.95f,    // HR (130°)
    -5.95f,   // HL (230°)
    -5.95f    // VL (310°)
};

static const float motorPosY[4] = {
    4.9915f,  // VR
    -4.9915f, // HR
    -4.9915f, // HL
    4.9915f   // VL
};

static const float motorAngle[4] = {
    motorVRversatz,
    motorHRversatz,
    motorHLversatz,
    motorVLversatz
};

// ----------------- Filter-/Rampenparameter -------------------
#define RATE_LIMIT 20.0f         // Maximale Änderung pro Sekunde (Einheiten/Sekunde)
#define LP_ALPHA   0.65f      // Low-Pass-Filter-Koeffizient (0.0 bis 1.0, kleiner = stärkeres Glätten)
#define DEAD_BAND  1         // Deadband, unterhalb dessen Werte als 0 gelten

class DriveSystem {
public:
    DriveSystem();

    // Berechnet die Motorsollwerte basierend auf Translation (x,y) und Rotation (r)
    void calcDrive(float vX, float vY, float r);
    
    // Führt eine zusätzliche Rotation der Eingangsvektoren durch (Matrix-Rotation)
    void calcDriveRotation(float x, float y, float r, float theta);
    
    // Setzt die berechneten Werte an die Motoren
    void drive();
    
    // Rampenratenbegrenzung: begrenzt den Übergang von current zu target um maximal rampRate

    void setMotor(int pinA, int pinB, int pinPWM, int speed);


private:
    // Setzt einen einzelnen Motor (Richtung und PWM) basierend auf dem übergebenen Speed
    
    // Begrenze den Speed auf den zulässigen PWM-Bereich (-255 bis +255)

private:
    int motorVR;
    int motorHR;
    int motorHL;
    int motorVL;
    
    // Zusätzliche Membervariablen für zeitbasierte Integration und Filterung:
    unsigned long lastUpdateTime; // Letzter Update-Zeitpunkt (Millis)
    float prevSpeed[4];    // Vorherige Sollwerte für die 4 Motoren
    float currentSpeed[4]; // Aktuell gefilterte Sollwerte für die 4 Motoren
};

#endif