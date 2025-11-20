#include "LidarProcessing.h"
#include "math.h"
#include <vector>
#include "GyroSystem.h"
#include "LD19.h"

// --- Globale Variablen ---
Vec2 Player = {0, 0}; 

LidarPoint points360[360];

// Histogramm-Arrays
int similarPointsX[FIELD_X_CM + 5][2]; 
int similarPointsY[FIELD_Y_CM + 5][2];

// Instanzen
LD19 lidarsensor;
extern GyroSystem gyro;

// --- Kalman Filter (Position in CM) ---
// Werte skaliert auf CM: 
// Messrauschen: 15.0 cm (statt 150 mm) - LiDAR trauen wir auf ca. 15cm genau
// Schätzfehler Init: 10.0 cm
// Prozessrauschen: 1.0 cm (statt 10 mm) - Dynamik des Roboters
SimpleKalman kalmanX(15.0f, 10.0f, 1.0f); 
SimpleKalman kalmanY(15.0f, 10.0f, 1.0f);
unsigned long lastKalmanTime = 0;

// --- Implementierung SimpleKalman ---
SimpleKalman::SimpleKalman(float mea_e, float est_e, float q) 
    : _err_measure(mea_e), _err_estimate(est_e), _q(q), _current_estimate(0), _last_estimate(0) {}

float SimpleKalman::update(float mea, float dt) {
    _err_estimate = _err_estimate + _q * dt; 
    _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
    _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    _err_estimate = (1.0f - _kalman_gain) * _err_estimate;
    _last_estimate = _current_estimate;
    return _current_estimate;
}

// --- Hilfsfunktionen ---
void resetSimilarPoints() {
    memset(similarPointsX, 0, sizeof(similarPointsX));
    memset(similarPointsY, 0, sizeof(similarPointsY));
}

void resetPoints360() {
    for(int i=0; i<360; i++) {
        points360[i].age++; 
        if(points360[i].age > 5) { 
             points360[i].distance = 0;
             points360[i].status = 999;
        }
    }
}

// -----------------------------------------------------------------
// Gyro Drift Korrektur (nutzt mm Punkte für Präzision)
// -----------------------------------------------------------------
void correctGyroDrift(int bestX_cm, int bestY_cm) {
    long sumX = 0, sumY = 0;
    long sumXY = 0, sumXX = 0, sumYY = 0;
    int count = 0;

    bool useVerticalWall = (abs(bestX_cm) > 0);
    // Ziel-Distanz in mm umrechnen, da points360 in mm sind
    int targetDistMm = abs(bestX_cm) * 10; 
    int toleranceMm = 150; 

    bool useHorizontalWall = false;
    if (!useVerticalWall && abs(bestY_cm) > 0) {
        useHorizontalWall = true;
        targetDistMm = abs(bestY_cm) * 10;
    }

    if (!useVerticalWall && !useHorizontalWall) return;

    for (int i = 0; i < 360; i++) {
        if (points360[i].distance == 0 || points360[i].status == 999) continue;
        
        int px = points360[i].x; // mm
        int py = points360[i].y; // mm

        if (useVerticalWall) {
            if (abs(abs(px) - targetDistMm) < toleranceMm) {
                sumX += px; sumY += py;
                sumXY += (long)px * py; sumYY += (long)py * py;
                count++;
            }
        } else {
            if (abs(abs(py) - targetDistMm) < toleranceMm) {
                sumX += px; sumY += py;
                sumXY += (long)px * py; sumXX += (long)px * px;
                count++;
            }
        }
    }

    if (count < 10) return;

    float angleErrorRad = 0.0f;

    if (useVerticalWall) {
        float numerator = (float)count * sumXY - (float)sumX * sumY;
        float denominator = (float)count * sumYY - (float)sumY * sumY;
        if (abs(denominator) > 1e-3) angleErrorRad = numerator / denominator; 
    } else {
        float numerator = (float)count * sumXY - (float)sumX * sumY;
        float denominator = (float)count * sumXX - (float)sumX * sumX;
        if (abs(denominator) > 1e-3) angleErrorRad = - (numerator / denominator); 
    }

    float errorDeg = angleErrorRad * (180.0f / PI);
    if (abs(errorDeg) > 10.0f) return; 

    float gain = 0.01f; 
    gyro.adjustOffset(errorDeg * gain); 
}

// -----------------------------------------------------------------
// EKF & Clustering Logik
// -----------------------------------------------------------------
void processPositionEKF() {
    resetSimilarPoints();

    // 1. Histogramm füllen
    for (int i = 0; i < 360; ++i) {
        if (points360[i].distance == 0 || points360[i].status == 999) continue;
        
        // Umrechnung mm -> cm für das Histogramm
        int xCm = points360[i].x / 10;
        int yCm = points360[i].y / 10;

        if (abs(xCm) <= FIELD_X_CM) {
            (xCm >= 0) ? similarPointsX[xCm][0]++ : similarPointsX[-xCm][1]++;
        }
        if (abs(yCm) <= FIELD_Y_CM) {
            (yCm >= 0) ? similarPointsY[yCm][0]++ : similarPointsY[-yCm][1]++;
        }
    }

    // 2. Wände suchen (Indices sind in CM)
    int highestX = 0, lowestX = 0;
    int highestY = 0, lowestY = 0;

    for (int i = FIELD_X_CM; i >= 0; --i) { if (similarPointsX[i][0] > 1) { highestX = i; break; } }
    for (int i = FIELD_X_CM; i >= 0; --i) { if (similarPointsX[i][1] > 1) { lowestX = -i; break; } }
    for (int i = FIELD_Y_CM; i >= 0; --i) { if (similarPointsY[i][0] > 1) { highestY = i; break; } }
    for (int i = FIELD_Y_CM; i >= 0; --i) { if (similarPointsY[i][1] > 1) { lowestY = -i; break; } }

    // Position berechnen IN CM
    // Hinweis: highestX und lowestX sind cm-Indices.
    // Beispiel: Wand bei 80cm, Wand bei -80cm -> (80 + (-80)) / 2 = 0 cm.
    // Beispiel: Wand bei 180cm (nur eine Seite sichtbar). highestX=180, lowestX=0.
    // Der EKF glättet den Sprung, wenn eine Wand verschwindet.
    // Wir nutzen hier KEIN * 10.0f mehr!
    
    float rawX = (highestX + lowestX) * 0.5f;
    float rawY = (highestY + lowestY) * 0.5f;

    // 3. Kalman Update Position (Werte sind jetzt cm)
    unsigned long now = millis();
    float dt = (now - lastKalmanTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f; if (dt > 0.1f) dt = 0.1f;
    lastKalmanTime = now;

    bool updateX = (highestX != 0 || lowestX != 0);
    bool updateY = (highestY != 0 || lowestY != 0);

    Player.x = updateX ? kalmanX.update(rawX, dt) : kalmanX.update(kalmanX.getEstimate(), dt);
    Player.y = updateY ? kalmanY.update(rawY, dt) : kalmanY.update(kalmanY.getEstimate(), dt);

    // 4. Gyro Drift korrigieren
    int bestWallX = (highestX != 0) ? highestX : lowestX;
    int bestWallY = (highestY != 0) ? highestY : lowestY;
    
    correctGyroDrift(bestWallX, bestWallY);
}


void lidaar() {
  lidarsensor.loop();
  
  double angleRadians = gyro.getAngleRadians();
  double cosTheta = cos(angleRadians);
  double sinTheta = sin(angleRadians);

  for (int i = 0; i < POINTCLOUD_SIZE; i++) {
    double lx = lidarsensor.lidar_points[i].y; 
    double ly = lidarsensor.lidar_points[i].x;

    if (lx == 0 && ly == 0) continue;

    // Transformation
    double xRotated = lx * cosTheta - ly * sinTheta;
    double yRotated = lx * sinTheta + ly * cosTheta;
    
    double xAdjusted = yRotated;
    double yAdjusted = xRotated * (-1);

    int angleDeg = (int)(atan2(yAdjusted, xAdjusted) * 180.0 / PI);
    if (angleDeg < 0) angleDeg += 360;
    int index = angleDeg % 360;
    
    int distance = (int)hypot(xAdjusted, yAdjusted);

    if (distance <= 4000 && distance >= 100) {
        points360[index].distance  = distance;
        points360[index].intensity = lidarsensor.lidar_points[i].intensity;
        points360[index].age       = 0;
        // Speichern in MM für interne Präzision (Drift-Korrektur)
        points360[index].x         = (int16_t)xAdjusted;
        points360[index].y         = (int16_t)yAdjusted;
        points360[index].status    = 0;
    }
  }

  resetPoints360();
  processPositionEKF();
}

void LidarBegin()
{
  lidarsensor.begin(&Serial1);
  kalmanX.setEstimate(0.0f);
  kalmanY.setEstimate(0.0f);
  lastKalmanTime = millis();
}