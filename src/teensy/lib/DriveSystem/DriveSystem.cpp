#include "DriveSystem.h"

// ------------------------ Hilfsfunktionen ------------------------

// clampSpeed: Beschränkt den übergebenen Speed auf den zulässigen Bereich (-MAX_PWM_OUTPUT  bis MAX_PWM_OUTPUT )



// setMotor: Steuert einen einzelnen Motor (Richtung und PWM) anhand des Speed-Werts
void DriveSystem::setMotor(int pinA, int pinB, int pinPWM, int speed) {
  
  if (speed == 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, 0);
    return;
  }
  
  // Vorwärtsbetrieb
  if (speed >= minSpeed) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    analogWrite(pinPWM, speed);
  }
  // Rückwärtsbetrieb
  else if (speed <= -minSpeed) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, abs(speed));
  }
  else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, 0);
  }
}

// ------------------------ Konstruktor ------------------------

DriveSystem::DriveSystem() {
  motorVR = motorHR = motorHL = motorVL = 0;
 
}

// ------------------------ calcDrive ------------------------


// In deiner DriveSystem.cpp Datei
void DriveSystem::calcDrive(float vX, float vY, float r) {
    const int MAX_PWM_OUTPUT = 180;


    // === SCHRITT 1: Berechne die benötigten Geschwindigkeits-Komponenten ===
    float transVR_comp = (vX * cos(motorVRversatz)) + (vY * sin(motorVRversatz));
    float transHR_comp = (vX * cos(motorHRversatz)) + (vY * sin(motorHRversatz));
    float transHL_comp = (vX * cos(motorHLversatz)) + (vY * sin(motorHLversatz));
    float transVL_comp = (vX * cos(motorVLversatz)) + (vY * sin(motorVLversatz));
    // Die Rotations-Komponente ist für alle Motoren gleich: r

    // === SCHRITT 2: Finde die maximale Anforderung und skaliere bei Bedarf ===
    // Wir finden den Motor, der am meisten "gestresst" wird, indem wir die Absolutbeträge
    // der Anforderungen für Translation und Rotation addieren.
    float max_demand = 0;
    max_demand = max(max_demand, abs(transVR_comp) + abs(r));
    max_demand = max(max_demand, abs(transHR_comp) + abs(r));
    max_demand = max(max_demand, abs(transHL_comp) + abs(r));
    max_demand = max(max_demand, abs(transVL_comp) + abs(r));

    // Wenn die maximale Anforderung unser Geschwindigkeitslimit (z.B. 200) übersteigt,
    // müssen wir alle Eingangs-Befehle proportional verkleinern.
    if (max_demand > maxSpeed) {
        float scale = (float)maxSpeed / max_demand;
        vX *= scale;
        vY *= scale;
        r *= scale;
        // Berechne die Translations-Komponenten neu mit den skalierten Werten
        transVR_comp = (vX * cos(motorVRversatz)) + (vY * sin(motorVRversatz));
        transHR_comp = (vX * cos(motorHRversatz)) + (vY * sin(motorHRversatz));
        transHL_comp = (vX * cos(motorHLversatz)) + (vY * sin(motorHLversatz));
        transVL_comp = (vX * cos(motorVLversatz)) + (vY * sin(motorVLversatz));
    }


    float final_speed_vr = transVR_comp + r;
    float final_speed_hr = transHR_comp + r;
    float final_speed_hl = transHL_comp + r;
    float final_speed_vl = transVL_comp + r;

    const float deadband = 4.0f;
    motorVR = (abs(final_speed_vr) < deadband) ? 0 : (final_speed_vr > 0 ? map(final_speed_vr, 0, maxSpeed, minSpeed, MAX_PWM_OUTPUT ) : map(final_speed_vr, -maxSpeed, 0, -MAX_PWM_OUTPUT , -minSpeed));
    motorHR = (abs(final_speed_hr) < deadband) ? 0 : (final_speed_hr > 0 ? map(final_speed_hr, 0, maxSpeed, minSpeed, MAX_PWM_OUTPUT ) : map(final_speed_hr, -maxSpeed, 0, -MAX_PWM_OUTPUT , -minSpeed));
    motorHL = (abs(final_speed_hl) < deadband) ? 0 : (final_speed_hl > 0 ? map(final_speed_hl, 0, maxSpeed, minSpeed, MAX_PWM_OUTPUT ) : map(final_speed_hl, -maxSpeed, 0, -MAX_PWM_OUTPUT , -minSpeed));
    motorVL = (abs(final_speed_vl) < deadband) ? 0 : (final_speed_vl > 0 ? map(final_speed_vl, 0, maxSpeed, minSpeed, MAX_PWM_OUTPUT ) : map(final_speed_vl, -maxSpeed, 0, -MAX_PWM_OUTPUT , -minSpeed));

    //     Serial.print("motorVR: "); Serial.print(motorVR);
    // Serial.print("\tmotorHR: "); Serial.print(motorHR);
    // Serial.print("\tmotorHL: "); Serial.print(motorHL);
    // Serial.print("\tmotorVL: "); Serial.println(motorVL);
}
// ------------------------ calcDriveRotation ------------------------
//
// Diese Funktion rotiert den Eingangsvektor (x,y) um den Winkel theta
// und ruft anschließend calcDrive mit den rotierten Werten auf.
void DriveSystem::calcDriveRotation(float x, float y, float r, float theta) {
  float rotatedX = x * cos(theta) - y * sin(theta);
  float rotatedY = x * sin(theta) + y * cos(theta);
  calcDrive(rotatedX, rotatedY, r);
}


// ------------------------ drive ------------------------
//
// Sendet die berechneten Motorwerte an die Hardware.
void DriveSystem::drive() {
  setMotor(motorVRpin[0], motorVRpin[1], motorVRpin[2], motorVR);
  setMotor(motorHRpin[0], motorHRpin[1], motorHRpin[2], motorHR);
  setMotor(motorHLpin[0], motorHLpin[1], motorHLpin[2], motorHL);
  setMotor(motorVLpin[0], motorVLpin[1], motorVLpin[2], motorVL);
}
