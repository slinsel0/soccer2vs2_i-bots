#include "PIDController.h"

PIDController::PIDController(float p, float i, float d, unsigned long timeStepMs, float outILimit)
  : _p(p), _i(i), _d(d), _timeStepMs(timeStepMs), _outILimit(outILimit),
    _prevError(0.0f), _integral(0.0f), _lastTime(millis()), _lastOutput(0.0f)
{
}




 void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
    _lastOutput = 0.0f;
}


float PIDController::update(float error) {
    unsigned long now = millis();
    unsigned long dt_ms = now - _lastTime;
    if (dt_ms < _timeStepMs) return _lastOutput;

    float dt = float(dt_ms) / 1000.0f; // Sekunden!
    _lastTime = now;

    // I-Anteil mit Anti-Windup
    _integral += error * dt;
    float iTerm = _i * _integral;
    iTerm = constrain(iTerm, -_outILimit, _outILimit);

    float d = (dt > 0.f) ? (error - _prevError) / dt : 0.f;
    _prevError = error;

    _lastOutput = _p * error + iTerm + _d * d;
    return _lastOutput;
}

float PIDController::updatePD(float error) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - _lastTime;
    
    if (elapsedTime >= _timeStepMs) {
        float deltaTime = float(elapsedTime);
        
        // P-Anteil
        float outP = _p * error;
        
        // D-Anteil (ohne Integralanteil)
        float derivative = (deltaTime > 0.0f) ? (error - _prevError) / deltaTime : 0.0f;
        float outD = _d * derivative;
        
        _prevError = error;
        _lastTime = currentTime;
        _lastOutput = outP + outD;
        
        return _lastOutput;
    }
    return _lastOutput;
}
