#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>

class PIDController {
  public:
    // Konstruktor: p, i, d, Aktualisierungsintervall in Millisekunden und I-Begrenzung
    PIDController(float p, float i, float d, unsigned long timeStepMs, float outILimit);

    // Vollständiger PID-Update
    float update(float error);

    // PD-Update (ohne Integralanteil)
    float updatePD(float error);

  private:
    float _p;
    float _i;
    float _d;
    unsigned long _timeStepMs;
    float _outILimit;

    float _prevError;
    float _integral;
    unsigned long _lastTime;
    float _lastOutput;
};

#endif // PIDCONTROLLER_H
