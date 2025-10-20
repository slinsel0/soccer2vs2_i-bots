#ifndef LIDARPROCESSING_H
#define LIDARPROCESSING_H

#include <Arduino.h>
#include <vector>
#include "LD19.h"

extern LD19 lidarsensor;



// --- Feldgrößen als Makros (in mm) ---
#define FIELD_X_MM 1820
#define FIELD_Y_MM 2430

// Umrechnung in cm
#define FIELD_X_CM (FIELD_X_MM / 10)
#define FIELD_Y_CM (FIELD_Y_MM / 10)

// --- LiDAR-Konstanten ---
#ifndef POINTCLOUD_SIZE
  #define POINTCLOUD_SIZE 400
#endif



// --- Strukturen ---
struct Vec2 {
  float x;
  float y;
};

#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
#endif

struct Vec2E {
  float x;
  float y;
  float probability;
};

// Roboterposition (mm)
extern Vec2 Player;

// LiDAR-Punkts (alle Maße in mm)
struct LidarPoint {
  int distance;    // Entfernung (mm)
  int intensity;
  int age;
  int x;           // x-Koordinate
  int y;           // y-Koordinate
  int status;
};



// Array mit 360 LiDAR-Punkten
extern LidarPoint points360[360];

// --- LiDAR-Funktionen ---
Vec2 Vec2EToVec2(const Vec2E &vec);
bool IsPointInCircle(Vec2 point, Vec2 center, float radius);

void resetPoint(int i);
void countCycle();
void filterPoints();
void resetSimilarPoints();
void clusterPoints();
void setBotX();
void setBotY();
void calculateBotPos();
void lidaar();
void LidarBegin();



#endif
