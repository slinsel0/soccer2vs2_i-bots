#ifndef LIDARPROCESSING_H
#define LIDARPROCESSING_H

#include <Arduino.h>
#include <vector>
#include "LD19.h"

extern LD19 lidarsensor;

// --- Feldgrößen (in mm für interne Vergleiche) ---
#define FIELD_X_MM 1820
#define FIELD_Y_MM 2430

// Umrechnung in cm für das Histogramm-Grid
#define FIELD_X_CM (FIELD_X_MM / 10)
#define FIELD_Y_CM (FIELD_Y_MM / 10)

// LiDAR-Konstanten
#ifndef POINTCLOUD_SIZE
  #define POINTCLOUD_SIZE 400
#endif

// --- Strukturen ---
#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 {
  float x;
  float y;
};
#endif

struct Vec2E {
  float x;
  float y;
  float probability;
};

// Roboterposition in CM (angepasst für main.cpp/outofbounce)
extern Vec2 Player;

// LiDAR-Punkt Struktur (intern weiter in mm für Präzision)
struct LidarPoint {
  int distance;    // mm
  uint8_t intensity;
  uint8_t age;     
  int16_t x;       // mm
  int16_t y;       // mm
  uint8_t status;  // 0=ok, 1=wand, 999=invalid
};

extern LidarPoint points360[360];

// --- Fast EKF Klasse (1D) ---
class SimpleKalman {
public:
    SimpleKalman(float mea_e, float est_e, float q);
    float update(float mea, float dt);
    float getEstimate() const { return _current_estimate; }
    void setEstimate(float est) { _current_estimate = est; }

private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
};

// --- Funktionen ---
Vec2 Vec2EToVec2(const Vec2E &vec);
bool IsPointInCircle(Vec2 point, Vec2 center, float radius);

void lidaar();      
void LidarBegin();  

#endif