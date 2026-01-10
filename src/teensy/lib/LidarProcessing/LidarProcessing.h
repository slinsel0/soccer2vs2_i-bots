#ifndef LIDARPROCESSING_H
#define LIDARPROCESSING_H

#include <Arduino.h>
#include <vector>
#include "LD19.h"

// --- Feldgrößen (in mm) ---
// RoboCup Junior Soccer Open Field 
#define FIELD_X_MM 1820
#define FIELD_Y_MM 2430

// --- Strukturen ---

// Fix für redefinition of struct Vec2
#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 {
  float x;
  float y;
};
#endif

// Externe Globals
extern Vec2 Player; // Roboterposition in mm (0,0 = Mitte)

// --- Funktionen ---
void LidarBegin();
void lidaar(); // Hauptschleife

#endif