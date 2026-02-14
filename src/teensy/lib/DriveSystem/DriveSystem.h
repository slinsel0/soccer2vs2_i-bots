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
  // - maxSpeedTrans: Max Translation (vX/vY)
  // - maxSpeedRot:   Max Rotation (r_cmd)
  // - minPwm:        Mindest-PWM gegen Haftreibung
  // - maxPwm:        Oberes PWM-Limit (standard 220)
  void setLimits(float maxSpeedTrans_, float maxSpeedRot_, int minPwm, int maxPwm);

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

  float getMaxSpeedTrans() const { return maxSpeedTrans; }
  float getMaxSpeedRot()   const { return maxSpeedRot; }
  int   getMinPwm()   const { return minSpeed; }
  int   getMaxPwm()   const { return MAX_PWM_OUTPUT; }
  float getDeadband() const { return deadband; }
  int   getSlewPerCycle() const { return slewPerCycle; }

private:
  // Interne Motor-Ansteuerung (Richtung + PWM)
  void setMotor(int pinA, int pinB, int pinPWM, int speed);

  // Matrix nach Winkel-/Radius-Änderungen neu aufbauen
  void rebuildMatrix();

  // ------------- Kinematik-Matrix -------------
  // Jede Zeile i: [cos(theta_i), sin(theta_i), 1.0]
  float M[4][3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };

  // ------------- Tuning / Parameter -------------
  // Oberes PWM-Limit (wird für Mapping genutzt)
  int   MAX_PWM_OUTPUT = 220;

  // Deadband in "v"-Einheiten
  float deadband = 0.3f;

  // max. PWM-Schritt pro drive()-Aufruf
  int   slewPerCycle = 220;

  // Mindest-PWM (um Haftreibung sicher zu überwinden)
  int   minSpeed = 26;

  // Max PWM für Translation (vX/vY) — direkt in PWM-Einheiten
  float maxSpeedTrans = 400.0f;

  // Max PWM für Rotation (r_cmd) — direkt in PWM-Einheiten
  float maxSpeedRot = 100.0f;

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