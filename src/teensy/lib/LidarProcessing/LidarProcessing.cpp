#include "LidarProcessing.h"
#include <math.h>
#include <vector>
#include "GyroSystem.h"
#include "LD19.h"

// --- KONFIGURATION ---
// Versatz des LiDARs zur Robotermitte in mm
// Positiv = Lidar ist vor/rechts der Mitte
// Negativ = Lidar ist hinter/links der Mitte
#define LIDAR_MOUNT_OFFSET_X 0.0f  
#define LIDAR_MOUNT_OFFSET_Y 0.0f 

// Glättungsfaktor (0.01 = sehr träge/glatt, 1.0 = kein Filter)
// Vorher 0.3 -> Jetzt 0.08 für mehr Ruhe im Stillstand
#define FILTER_ALPHA 0.08f 

// --- Globale Variablen ---
Vec2 Player = {0.0f, 0.0f}; // Startposition Mitte (in mm)

// LiDAR Instanz
LD19 lidarsensor;
extern GyroSystem gyro;

// Tracking Variablen für Plausibilitäts-Check
static float lastValidX = 0.0f;
static float lastValidY = 0.0f;

// Konstanten für Filterung
const int   MIN_POINTS_FOR_WALL = 25; // Mindestens X Punkte für eine gültige Wand
const float WALL_WINDOW_CM = 15.0f; // Etwas enger tolerieren (15cm) für weniger Noise

void LidarBegin() {
  lidarsensor.begin(&Serial1);
  // Initialannahme: Wir starten ungefähr in der Mitte
  lastValidX = 0;
  lastValidY = 0;
}

/**
 * Kernfunktion zur Positionsbestimmung.
 */
void processLidarData() {
  // 1. Gyro Daten holen
  double angleRadians = gyro.getAngleRadians();
  
  // Trigonoemtrie vorbereiten
  double cosTheta = cos(angleRadians);
  double sinTheta = sin(angleRadians);

  // Akkumulatoren
  float sumX = 0.0f;
  float weightX = 0.0f;
  float sumY = 0.0f;
  float weightY = 0.0f;

  int pointsCountedX = 0;
  int pointsCountedY = 0;

  // Erwartete Wandpositionen (Global)
  float fieldHalfX = FIELD_X_MM / 2.0f;
  float fieldHalfY = FIELD_Y_MM / 2.0f;

  // Iteriere durch LiDAR Punkte
  for (int i = 0; i < POINTCLOUD_SIZE; i++) {
    
    // --- DEINE TRANSFORMATION ---
    // Rohdaten vom Sensor
    double x = lidarsensor.lidar_points[i].y; 
    double y = lidarsensor.lidar_points[i].x;
    
    if (x == 0 && y == 0) continue;

    // Distanz Filter (Quadratisch für Performance)
    double distSq = x*x + y*y;
    if (distSq < 150*150) continue; // Blindzone 15cm
    if (distSq > 3000*3000) continue; // Max Range 3m
    
    if (lidarsensor.lidar_points[i].intensity < 100) continue; 

    // Rotation (Welt-Koordinaten relativ zum Lidar-Sensor)
    double xRotated = x * cosTheta - y * sinTheta;
    double yRotated = x * sinTheta + y * cosTheta;
    
    // Achsen-Mapping (an Roboter-System angepasst)
    // Lidar System -> Roboter System
    double xRobotFrame = yRotated;
    double yRobotFrame = xRotated * (-1);
    
    // Korrektur um Montage-Offset (Lidar Position -> Roboter Zentrum)
    // Wenn Lidar bei x=+50mm steht und Wand bei +900mm misst er 850mm.
    // Wir wollen aber wissen wo das ZENTRUM ist.
    // Zentrum = WandPos - (Gemessen + Offset) ?
    // Einfacher: Wir transformieren den Punkt so, als ob er vom Zentrum gemessen wurde.
    xRobotFrame += LIDAR_MOUNT_OFFSET_X; 
    yRobotFrame += LIDAR_MOUNT_OFFSET_Y;
    // -----------------------------

    float rotX = (float)xRobotFrame;
    float rotY = (float)yRobotFrame;

    // --- X-ACHSE ANALYSE ---
    // Hypothese 1: Punkt ist rechte Wand (+X) -> Roboter ist bei (WandPos - Distanz)
    float measPosX_Right = fieldHalfX - rotX;
    
    // Hypothese 2: Punkt ist linke Wand (-X) -> Roboter ist bei (-WandPos - Distanz)
    float measPosX_Left = -fieldHalfX - rotX;

    // Gating: Passt das zur letzten Position? (in mm umgerechnet)
    bool matchRight = fabsf(measPosX_Right - lastValidX) < (WALL_WINDOW_CM * 10.0f);
    bool matchLeft  = fabsf(measPosX_Left  - lastValidX) < (WALL_WINDOW_CM * 10.0f);

    // Initialisierung wenn wir noch bei 0.0 stehen und keine Matches haben
    if (fabsf(lastValidX) < 1.0f && !matchRight && !matchLeft) {
         // Wir trauen dem Sensor initial, wenn der Wert im Feld liegt
         if (fabsf(measPosX_Right) < fieldHalfX) matchRight = true;
         if (fabsf(measPosX_Left) < fieldHalfX) matchLeft = true;
    }

    if (matchRight) {
        sumX += measPosX_Right;
        weightX += 1.0f;
        pointsCountedX++;
    }
    if (matchLeft) {
        sumX += measPosX_Left;
        weightX += 1.0f;
        pointsCountedX++;
    }

    // --- Y-ACHSE ANALYSE ---
    float measPosY_Front = fieldHalfY - rotY;
    float measPosY_Back  = -fieldHalfY - rotY;

    bool matchFront = fabsf(measPosY_Front - lastValidY) < (WALL_WINDOW_CM * 10.0f);
    bool matchBack  = fabsf(measPosY_Back  - lastValidY) < (WALL_WINDOW_CM * 10.0f);

    if (fabsf(lastValidY) < 1.0f && !matchFront && !matchBack) {
         if (fabsf(measPosY_Front) < fieldHalfY) matchFront = true;
         if (fabsf(measPosY_Back) < fieldHalfY) matchBack = true;
    }

    if (matchFront) {
        sumY += measPosY_Front;
        weightY += 1.0f;
        pointsCountedY++;
    }
    if (matchBack) {
        sumY += measPosY_Back;
        weightY += 1.0f;
        pointsCountedY++;
    }
  }

  // 3. Update Position (Glättung)
  
  if (pointsCountedX > MIN_POINTS_FOR_WALL) {
      float newX_mm = sumX / weightX;
      
      // Filter anwenden
      float rawNewX = (lastValidX) * (1.0f - FILTER_ALPHA) + newX_mm * FILTER_ALPHA;
      
      // Sicherheit: Im Feld bleiben (+ Puffer für leichte Ausflüge)
      Player.x = constrain(rawNewX, -fieldHalfX - 50.0f, fieldHalfX + 50.0f);
      lastValidX = Player.x;
  }
  
  if (pointsCountedY > MIN_POINTS_FOR_WALL) {
      float newY_mm = sumY / weightY;
      
      float rawNewY = (lastValidY) * (1.0f - FILTER_ALPHA) + newY_mm * FILTER_ALPHA;
      
      Player.y = constrain(rawNewY, -fieldHalfY - 50.0f, fieldHalfY + 50.0f);
      lastValidY = Player.y;
  }
}

void lidaar() {
  lidarsensor.loop();
  processLidarData();
}