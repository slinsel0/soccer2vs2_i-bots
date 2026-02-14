#include "LidarProcessing.h"
#include <math.h>
#include <vector>
#include "GyroSystem.h"
#include "LD19.h"

// --- KONFIGURATION ---
// Versatz des LiDARs zur Robotermitte in mm
#define LIDAR_MOUNT_OFFSET_X 0.0f  
#define LIDAR_MOUNT_OFFSET_Y 0.0f 

// Mindestanzahl gültiger Punkte für eine Positionsberechnung
#define MIN_VALID_POINTS 10

// --- Globale Variablen ---
Vec2 Player = {0.0f, 0.0f};

// LiDAR Instanz
LD19 lidarsensor;
extern GyroSystem gyro;

void LidarBegin() {
  lidarsensor.begin(&Serial1);
}

/**
 * Kernfunktion zur Positionsbestimmung.
 * Simpel & robust: Max/Min der rotierten Punktwolke bestimmen,
 * daraus die Roboterposition ableiten.
 * 
 * Logik: Wenn maxX die rechte Wand ist und minX die linke Wand,
 * dann ist die Roboterposition: posX = -(maxX + minX) / 2
 * (analog für Y-Achse)
 */
void processLidarData() {
  // 1. Gyro Daten holen
  double angleRadians = gyro.getAngleRadians();
  double cosTheta = cos(angleRadians);
  double sinTheta = sin(angleRadians);

  // Feldgrenzen
  float fieldHalfX = FIELD_X_MM / 2.0f;
  float fieldHalfY = FIELD_Y_MM / 2.0f;

  // Max/Min Tracker
  float maxX = -99999.0f;
  float minX =  99999.0f;
  float maxY = -99999.0f;
  float minY =  99999.0f;

  int validPoints = 0;

  // Iteriere durch LiDAR Punkte
  for (int i = 0; i < POINTCLOUD_SIZE; i++) {
    
    // Rohdaten vom Sensor
    double x = lidarsensor.lidar_points[i].y; 
    double y = lidarsensor.lidar_points[i].x;
    
    if (x == 0 && y == 0) continue;

    // Distanz Filter
    double distSq = x*x + y*y;
    if (distSq < 150*150) continue;   // Blindzone 15cm
    if (distSq > 2600*2600) continue;  // Max Range 3m
    
    // if (lidarsensor.lidar_points[i].intensity < 100) continue; 

    // Rotation in Welt-Koordinaten
    double xRotated = x * cosTheta - y * sinTheta;
    double yRotated = x * sinTheta + y * cosTheta;
    
    // Achsen-Mapping: Lidar System -> Roboter System
    double xRobotFrame = yRotated;
    double yRobotFrame = xRotated * (-1);
    
    // Montage-Offset korrigieren
    xRobotFrame += LIDAR_MOUNT_OFFSET_X; 
    yRobotFrame += LIDAR_MOUNT_OFFSET_Y;

    float rotX = (float)xRobotFrame;
    float rotY = (float)yRobotFrame;

    // Max/Min updaten
    if (rotX > maxX) maxX = rotX;
    if (rotX < minX) minX = rotX;
    if (rotY > maxY) maxY = rotY;
    if (rotY < minY) minY = rotY;

    validPoints++;
  }

  // 2. Position aus Max/Min berechnen
  //    maxX = Abstand zur rechten Wand,  minX = Abstand zur linken Wand (negativ)
  //    Roboter-Position = -(maxX + minX) / 2
  if (validPoints >= MIN_VALID_POINTS) {
      float posX = -(maxX + minX) / 2.0f;
      float posY = -(maxY + minY) / 2.0f;

      // Sicherheit: Im Feld bleiben
      Player.x = constrain(posX, -fieldHalfX, fieldHalfX);
      Player.y = constrain(posY, -fieldHalfY, fieldHalfY);
  }
}

void lidaar() {
  lidarsensor.loop();
  processLidarData();
}