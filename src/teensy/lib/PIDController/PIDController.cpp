#include "PIDController.h"

PIDController::PIDController(float p, float i, float d, unsigned long timeStepMs, float outILimit)
  : _p(p), _i(i), _d(d), _timeStepMs(timeStepMs), _outILimit(outILimit),
    _prevError(0.0f), _integral(0.0f), _lastTime(millis()), _lastOutput(0.0f)
{
}

float PIDController::update(float error) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - _lastTime;
    
    if (elapsedTime >= _timeStepMs) {
        // Verwende deltaTime in Millisekunden (keine Umrechnung in Sekunden)
        float deltaTime = float(elapsedTime);

        // P-Anteil
        float outP = _p * error;
        
        // I-Anteil (Anti-Windup, Tuning: _i in [pro ms])
        _integral += error * deltaTime;
        if (_integral > _outILimit)
            _integral = _outILimit;
        else if (_integral < -_outILimit)
            _integral = -_outILimit;
        float outI = _i * _integral;
        
        // D-Anteil (Tuning: _d in [pro ms])
        float derivative = (deltaTime > 0.0f) ? (error - _prevError) / deltaTime : 0.0f;
        float outD = _d * derivative;
        
        // Zustände aktualisieren
        _prevError = error;
        _lastTime = currentTime;
        _lastOutput = outP + outI + outD;
        
        return _lastOutput;
    }
    // Ist der Update-Zeitraum noch nicht erreicht, wird der letzte berechnete Wert zurückgegeben.
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
