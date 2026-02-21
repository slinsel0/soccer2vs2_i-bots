#include "DriveSystem.h"

// ------------------------ Hilfsfunktionen ------------------------

// clampSpeed: Beschränkt den übergebenen Speed auf den zulässigen Bereich (-MAX_PWM_OUTPUT  bis MAX_PWM_OUTPUT )


namespace {
  inline float clampf(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }

  inline float lmapf(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) return out_min;
    float t = (x - in_min) / (in_max - in_min);
    return out_min + t * (out_max - out_min);
  }
}


// setMotor: Steuert einen einzelnen Motor (Richtung und PWM) anhand des Speed-Werts
void DriveSystem::setMotor(int pinA, int pinB, int pinPWM, int speed) {



  // Mindest-PWM gegen Haftreibung

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

  // Matrix: Projektion (cos/sin) + Rotations-Spalte (direkt 1.0)
  M[0][0] = cos(motorVRversatz); M[0][1] = sin(motorVRversatz); M[0][2] = 1.0f;
  M[1][0] = cos(motorHRversatz); M[1][1] = sin(motorHRversatz); M[1][2] = 1.0f;
  M[2][0] = cos(motorHLversatz); M[2][1] = sin(motorHLversatz); M[2][2] = 1.0f;
  M[3][0] = cos(motorVLversatz); M[3][1] = sin(motorVLversatz); M[3][2] = 1.0f;
}


// ------------------------ calcDrive ------------------------


void DriveSystem::calcDrive(float vX, float vY, float r) {

  vX *= -1;  // 1) Rotation begrenzen
 if (r > maxRotation) r = maxRotation;
  if (r < (maxRotation * -1)) r = maxRotation * -1;

  // 2) Translation per Matrix berechnen (cos/sin Projektion)
  motorVR = (int)lrintf(M[0][0]*vX + M[0][1]*vY);
  motorHR = (int)lrintf(M[1][0]*vX + M[1][1]*vY);
  motorHL = (int)lrintf(M[2][0]*vX + M[2][1]*vY);
  motorVL = (int)lrintf(M[3][0]*vX + M[3][1]*vY);




  //  int availableSpace = maxSpeed - abs((int)r) - minSpeed;
  // if (availableSpace < 0) availableSpace = 0; // Fallback
  // 3) Auf maxSpeed skalieren (Verhältnis beibehalten = gleiche Fahrrichtung)
    int motorMAX = max(max(abs(motorVR), abs(motorHR)), max(abs(motorHL), abs(motorVL)));
  if (motorMAX > maxSpeed) {
    motorVR = motorVR * maxSpeed / motorMAX;
    motorHR = motorHR * maxSpeed / motorMAX;
    motorHL = motorHL * maxSpeed / motorMAX;
    motorVL = motorVL * maxSpeed / motorMAX;
  }
  // 4) Rotation NACHHER addieren (unabhängig von Translation)
  motorVR += r;
  motorHR += r;
  motorHL += r;
  motorVL += r;


  // 5) minSpeed Offset (Treiber-Totzone überwinden)
  if (motorVR > 0) motorVR += minSpeed; else motorVR -= minSpeed;
  if (motorHR > 0) motorHR += minSpeed; else motorHR -= minSpeed;
  if (motorHL > 0) motorHL += minSpeed; else motorHL -= minSpeed;
  if (motorVL > 0) motorVL += minSpeed; else motorVL -= minSpeed;

  // 6) Zu kleine Werte auf 0 (Zuckungen vermeiden)
  if (abs(motorVR) < (minSpeed + 0)) motorVR = 0;
  if (abs(motorHR) < (minSpeed + 0)) motorHR = 0;
  if (abs(motorHL) < (minSpeed + 0)) motorHL = 0;
  if (abs(motorVL) < (minSpeed + 0)) motorVL = 0;
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
static inline int applySlew(int target, int prev, int step) {
  if (target > prev + step) return prev + step;
  if (target < prev - step) return prev - step;
  return target;
}

void DriveSystem::drive() {
  // sanft machen
  int outVR = applySlew(motorVR, prevVR, slewPerCycle);
  int outHR = applySlew(motorHR, prevHR, slewPerCycle);
  int outHL = applySlew(motorHL, prevHL, slewPerCycle);
  int outVL = applySlew(motorVL, prevVL, slewPerCycle);

  setMotor(motorVRpin[0], motorVRpin[1], motorVRpin[2], outVR);
  setMotor(motorHRpin[0], motorHRpin[1], motorHRpin[2], outHR);
  setMotor(motorHLpin[0], motorHLpin[1], motorHLpin[2], outHL);
  setMotor(motorVLpin[0], motorVLpin[1], motorVLpin[2], outVL);


  // Serial.print(">MOTORVR:");      Serial.println(outVR);
  // Serial.print(">MOTORHR:");      Serial.println(outHR);
  // Serial.print(">MOTORHL:");      Serial.println(outHL);
  // Serial.print(">MOTORVL:");      Serial.println(outVL);

  prevVR = outVR; prevHR = outHR; prevHL = outHL; prevVL = outVL;
}