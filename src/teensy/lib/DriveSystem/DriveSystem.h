#pragma once
#include <Arduino.h>
#include <math.h>


// ------------------------------------------------------------
// DriveSystem.h
// Omnidirektionales 4-Rad-Drive-System (XY + Rotation)
// - Matrix-Kinematik (cos/sin)
// - L∞-Skalierung auf Radebene
// - Deadband + lineares Float-Mapping -> PWM
// - Slew-Rate-Limit pro drive()-Aufruf
// ------------------------------------------------------------

class DriveSystem {
public:
  // --- Konstruktoren ---
  DriveSystem();
  DriveSystem(const int vrPins[3],
              const int hrPins[3],
              const int hlPins[3],
              const int vlPins[3],
              float vrAngle, float hrAngle, float hlAngle, float vlAngle);

  // --- Setup / Tuning ---
  void setMotorPins(const int vrPins[3],
                    const int hrPins[3],
                    const int hlPins[3],
                    const int vlPins[3]);

  // Winkel der Radtangentialrichtungen (in Rad), z. B. 45°, 135°, -135°, -45° => in Rad umrechnen
  void setMotorAngles(float vrAngle, float hrAngle, float hlAngle, float vlAngle);

  // Limits anpassen:
  // - maxSpeed:    Max Translation PWM
  // - maxRotation: Max Rotation PWM
  // - minPwm:      Mindest-PWM gegen Haftreibung
  // - maxPwm:      Oberes PWM-Limit (standard 220)
  void setLimits(int maxSpeed_, int maxRotation_, int minPwm, int maxPwm);

  // Deadband in "v"-Einheiten (kleine Werte auf 0 setzen)
  void setDeadband(float db);

  // Max. PWM-Änderung pro drive()-Aufruf (sanfte Übergänge)
  void setSlewPerCycle(int step);

  // --- Kinematik / Ausgabe ---
  // Berechnet Radbefehle aus (vX, vY, r) – r ist Winkelgeschwindigkeit
  void calcDrive(float vX, float vY, float r);

  // Dreht Eingangsvektor (x,y) um theta (in Rad) und ruft calcDrive()
  void calcDriveRotation(float x, float y, float r, float theta);

  // Sendet Motorwerte an die Hardware (inkl. Slew-Rate-Limit)
  void drive();

  // --- Debug / Zugriff ---
  int  getMotorVR() const { return motorVR; }
  int  getMotorHR() const { return motorHR; }
  int  getMotorHL() const { return motorHL; }
  int  getMotorVL() const { return motorVL; }

  int   getMaxSpeed()    const { return maxSpeed; }
  int   getMaxRotation() const { return maxRotation; }
  int   getMinPwm()   const { return minSpeed; }
  int   getMaxPwm()   const { return MAX_PWM_OUTPUT; }
  float getDeadband() const { return deadband; }
  int   getSlewPerCycle() const { return slewPerCycle; }

    void setMotor(int pinA, int pinB, int pinPWM, int speed);


private:
  // Interne Motor-Ansteuerung (Richtung + PWM)

  // Matrix nach Winkel-/Radius-Änderungen neu aufbauen
  void rebuildMatrix();

  // ------------- Kinematik-Matrix -------------
  // Jede Zeile i: [cos(theta_i), sin(theta_i), 1.0]
  float M[4][3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };

  // ------------- Tuning / Parameter -------------
  // Oberes PWM-Limit (wird für Mapping genutzt)
  int   MAX_PWM_OUTPUT = 220;

  // Deadband in "v"-Einheiten
  float deadband = 0.0f;

  // max. PWM-Schritt pro drive()-Aufruf
  int   slewPerCycle = 220;

  // Mindest-PWM (um Haftreibung sicher zu überwinden)
  int   minSpeed = 26;

  // Max Translation (PWM-Einheiten)
  int maxSpeed = 120;

  // Max Rotation (PWM-Einheiten)
  int maxRotation = 80;

  // ------------- Pins (je Motor: A, B, PWM) -------------
  int motorVRpin[3] = {3, 4, 15}; 
  int motorHRpin[3] = {5, 6, 19}; 
  int motorHLpin[3] = {10, 7, 14};
  int motorVLpin[3] = {12, 11, 18};

  // ------------- Radtangential-Winkel (in Rad) -------------
  float motorVRversatz = (PI * 50.0f  / 180.0f);
  float motorHRversatz = (PI * 130.0f / 180.0f);
  float motorHLversatz = (PI * 230.0f / 180.0f);
  float motorVLversatz = (PI * 310.0f / 180.0f);

  // ------------- Aktuelle Motor-Sollwerte (signed PWM) -------------
  int motorVR = 0, motorHR = 0, motorHL = 0, motorVL = 0;

  // ------------- Vorherige PWM (für Slew-Rate) -------------
  int prevVR = 0, prevHR = 0, prevHL = 0, prevVL = 0;
};