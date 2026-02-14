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
  const int HOLD_PWM = 26;           // hält Treiber „wach“ (wie bei dir)
  int pwm = abs(speed);

  if (pwm == 0) {
    // neutral / leichter Hold
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    analogWrite(pinPWM, HOLD_PWM);
    return;
  }

  // Mindest-PWM gegen Haftreibung
  if (pwm < minSpeed) pwm = minSpeed;

  if (speed > 0) {
    digitalWrite(pinA, LOW);
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

  // Matrix: Projektion (cos/sin) + Rotations-Spalte (direkt 1.0)
  M[0][0] = cos(motorVRversatz); M[0][1] = sin(motorVRversatz); M[0][2] = 1.0f;
  M[1][0] = cos(motorHRversatz); M[1][1] = sin(motorHRversatz); M[1][2] = 1.0f;
  M[2][0] = cos(motorHLversatz); M[2][1] = sin(motorHLversatz); M[2][2] = 1.0f;
  M[3][0] = cos(motorVLversatz); M[3][1] = sin(motorVLversatz); M[3][2] = 1.0f;
}


// ------------------------ calcDrive ------------------------


void DriveSystem::calcDrive(float vX, float vY, float r_cmd) {
  // 1) Translation und Rotation getrennt berechnen
  float wT[4], wR[4];
  const float c[4] = { cos(motorVRversatz), cos(motorHRversatz), cos(motorHLversatz), cos(motorVLversatz) };
  const float s[4] = { sin(motorVRversatz), sin(motorHRversatz), sin(motorHLversatz), sin(motorVLversatz) };

  for (int i=0;i<4;++i) {
    wT[i] = c[i]*vX + s[i]*vY;
    wR[i] = r_cmd;
  }

  // 2) Translation auf maxSpeedTrans clampen
  float maxT = 0.f; for (int i=0;i<4;++i) maxT = fmaxf(maxT, fabsf(wT[i]));
  if (maxT > maxSpeedTrans && maxSpeedTrans > 0) {
    float s_t = maxSpeedTrans / maxT;
    for (int i=0;i<4;++i) wT[i] *= s_t;
  }

  // 3) Rotation auf maxSpeedRot clampen
  float maxR = 0.f; for (int i=0;i<4;++i) maxR = fmaxf(maxR, fabsf(wR[i]));
  if (maxR > maxSpeedRot && maxSpeedRot > 0) {
    float s_r = maxSpeedRot / maxR;
    for (int i=0;i<4;++i) wR[i] *= s_r;
  }

  // 4) Zusammenmischen
  float w[4];
  for (int i=0;i<4;++i) w[i] = wT[i] + wR[i];

  // 5) PWM: direkt den Wert als PWM nutzen, auf MAX_PWM_OUTPUT deckeln
  auto toPWM = [&](float v)->int {
    if (fabsf(v) < deadband) return 0;
    int pwm = (int)lrintf(clampf(fabsf(v), (float)minSpeed, (float)MAX_PWM_OUTPUT));
    return v>=0 ? pwm : -pwm;
  };

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