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
  const int HOLD_PWM = 39;           // hält Treiber „wach“ (wie bei dir)
  int pwm = speed;

  if (pwm == 0) {
    // neutral / leichter Hold
     digitalWrite(pinB, HIGH);
    // digitalWrite(pinB, HIGH);
    analogWrite(pinPWM,50 );
    return;
  }

  // Mindest-PWM gegen Haftreibung
  if (pwm < minSpeed) pwm = minSpeed;

  if (speed > 0) {
    digitalWrite(pinB, HIGH);
    analogWrite(pinPWM, pwm);
  } else {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, pwm);
  }
}


// ------------------------ Konstruktor ------------------------

DriveSystem::DriveSystem() {
  motorVR = motorHR = motorHL = motorVL = 0;

  // Matrix: gleiche Projektion wie bei dir (cos/sin), plus Rotations-Spalte
  M[0][0] = cos(motorVRversatz); M[0][1] = sin(motorVRversatz); M[0][2] = rot_to_lin;
  M[1][0] = cos(motorHRversatz); M[1][1] = sin(motorHRversatz); M[1][2] = rot_to_lin;
  M[2][0] = cos(motorHLversatz); M[2][1] = sin(motorHLversatz); M[2][2] = rot_to_lin;
  M[3][0] = cos(motorVLversatz); M[3][1] = sin(motorVLversatz); M[3][2] = rot_to_lin;
}


// ------------------------ calcDrive ------------------------


void DriveSystem::calcDrive(float vX, float vY, float r_cmd) {
  // 1) getrennt rechnen
  float wT[4], wR[4];
  const float c[4] = { cos(motorVRversatz), cos(motorHRversatz), cos(motorHLversatz), cos(motorVLversatz) };
  const float s[4] = { sin(motorVRversatz), sin(motorHRversatz), sin(motorHLversatz), sin(motorVLversatz) };

  for (int i=0;i<4;++i) {
    wT[i] = c[i]*vX + s[i]*vY;          // Translation pro Rad
    wR[i] = rot_to_lin * r_cmd;         // Rotation pro Rad (gleich für alle)
  }

  // 2) Rotation priorisieren
  float maxR = 0.f; for (int i=0;i<4;++i) maxR = fmaxf(maxR, fabsf(wR[i]));
  float s_rot = (maxR > maxSpeed && maxSpeed>0) ? (maxSpeed / maxR) : 1.f;
  for (int i=0;i<4;++i) wR[i] *= s_rot;

  // 3) Rest-Headroom für Translation berechnen
  float s_trans = 1.f;
  for (int i=0;i<4;++i) {
    float hi = maxSpeed - fabsf(wR[i]);
    if (fabsf(wT[i]) > 1e-6f) {
      s_trans = fminf(s_trans, clampf(hi / fabsf(wT[i]), 0.f, 1.f));
    }
  }

  // 4) mischen + Deadband + PWM-Mapping (deine float-Map)
  auto toPWM = [&](float v)->int {
    if (fabsf(v) < deadband) return 0;
    float mag = lmapf(fabsf(v), 0.f, maxSpeed, (float)minSpeed, (float)MAX_PWM_OUTPUT);
    int pwm = (int)lrintf(clampf(mag, (float)minSpeed, (float)MAX_PWM_OUTPUT));
    return v>=0 ? pwm : -pwm;
  };

  float w[4];
  for (int i=0;i<4;++i) w[i] = wR[i] + s_trans*wT[i];

  motorVR = toPWM(w[0]);
  motorHR = toPWM(w[1]);
  motorHL = toPWM(w[2]);
  motorVL = toPWM(w[3]);
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

  prevVR = outVR; prevHR = outHR; prevHL = outHL; prevVL = outVL;
}
