#include "LidarProcessing.h"
#include <math.h>
#include <vector>
#include "GyroSystem.h"
#include "LD19.h"

// --- KONFIGURATION ---
#define LIDAR_MOUNT_OFFSET_X 0.0f  
#define LIDAR_MOUNT_OFFSET_Y 0.0f 

// Lidar Distanz-Filter
#define MIN_DIST_SQ (150.0f * 150.0f)     
#define MAX_DIST_SQ (3200.0f * 3200.0f)   

// Histogramm Konfiguration (optimiert für schnelle Bewegungen)
#define BIN_SIZE_MM 40.0f                 
#define HISTOGRAM_HALF_RANGE 3500.0f      
#define NUM_BINS (int((HISTOGRAM_HALF_RANGE * 2.0f) / BIN_SIZE_MM))
#define PEAK_THRESHOLD 3                  

// Globale Roboterposition
Vec2 Player = {0.0f, 0.0f};

// Externe Instanzen
LD19 lidarsensor;
extern GyroSystem gyro;

// Speicher für Histogramme
int histX[NUM_BINS];
int histY[NUM_BINS];

// --- SENSOR FUSION VARIABLEN ---
static float est_px = 0.0f;     // Geschätzte Position X in mm
static float est_py = 0.0f;     // Geschätzte Position Y in mm
static float est_vx = 0.0f;     // Geschätzte Geschwindigkeit X in mm/s
static float est_vy = 0.0f;     // Geschätzte Geschwindigkeit Y in mm/s

static uint32_t last_fusion_time_us = 0;
static uint32_t last_correction_time_us = 0;

void LidarBegin() {
  lidarsensor.begin(&Serial1);
}

void processLidarData() {
  uint32_t now_us = micros();
  
  // ==========================================================
  // 1. IMU PREDICTION (Hochfrequente Vorhersage)
  // ==========================================================
  if (last_fusion_time_us == 0) {
      last_fusion_time_us = now_us;
      last_correction_time_us = now_us;
      return;
  }
  
  // Delta-Zeit berechnen (in Sekunden)
  float dt = (now_us - last_fusion_time_us) / 1000000.0f;
  if (dt > 0.05f) dt = 0.005f; // Schutz vor riesigen Sprüngen bei Lags
  last_fusion_time_us = now_us;

  float yaw_rad = gyro.getAngleRadians();
  
  // Beschleunigung holen und in mm/s^2 umrechnen (m/s^2 * 1000)
  // WICHTIG: Prüfe im Teleplot, ob die Richtung stimmt! Ggf. ein Minus (-) vorsetzen!
  float a_local_x = gyro.getAccelX() * 1000.0f; 
  float a_local_y = gyro.getAccelY() * 1000.0f;

  float cos_y = cosf(yaw_rad);
  float sin_y = sinf(yaw_rad);
  
  // Lokale Beschleunigung ins Spielfeld-Koordinatensystem rotieren
  float a_global_x = a_local_x * cos_y - a_local_y * sin_y;
  float a_global_y = a_local_x * sin_y + a_local_y * cos_y;

  // Integrieren (Position blinden Auges weiterschätzen)
  est_vx += a_global_x * dt;
  est_vy += a_global_y * dt;
  est_px += est_vx * dt;
  est_py += est_vy * dt;

  // Künstliche Reibung: dämpft den Geschwindigkeits-Drift
  est_vx *= 0.98f;
  est_vy *= 0.98f;


  // ==========================================================
  // 2. LIDAR BERECHNUNG (Histogramm für robuste Wände)
  // ==========================================================
  memset(histX, 0, sizeof(histX));
  memset(histY, 0, sizeof(histY));
  int validPoints = 0;

  for (int i = 0; i < POINTCLOUD_SIZE; i++) {
    double x = lidarsensor.lidar_points[i].y; 
    double y = lidarsensor.lidar_points[i].x;
    
    if (x == 0 && y == 0) continue;

    double distSq = x*x + y*y;
    if (distSq < MIN_DIST_SQ || distSq > MAX_DIST_SQ) continue;   
    
    // Punkt-spezifischen Winkel anwenden (Rolling-Shutter Fix)
    float p_angle = lidarsensor.lidar_points[i].robot_angle_rad;
    float cosTheta = cosf(p_angle);
    float sinTheta = sinf(p_angle);

    double xRotated = x * cosTheta - y * sinTheta;
    double yRotated = x * sinTheta + y * cosTheta;
    
    double xRobotFrame = yRotated + LIDAR_MOUNT_OFFSET_X;
    double yRobotFrame = (xRotated * -1.0) + LIDAR_MOUNT_OFFSET_Y;

    int binX = (int)((xRobotFrame + HISTOGRAM_HALF_RANGE) / BIN_SIZE_MM);
    int binY = (int)((yRobotFrame + HISTOGRAM_HALF_RANGE) / BIN_SIZE_MM);

    if (binX >= 0 && binX < NUM_BINS) histX[binX]++;
    if (binY >= 0 && binY < NUM_BINS) histY[binY]++;
    validPoints++;
  }


  // ==========================================================
  // 3. CORRECTION (Lidar fängt die IMU sanft ein)
  // ==========================================================
  // LD19 rotiert mit 10 Hz -> Wir machen den Abgleich nur alle 100 ms!
  if (validPoints >= 20 && (now_us - last_correction_time_us) > 100000) {
      last_correction_time_us = now_us;

      float leftWallX = 0, rightWallX = 0;
      float bottomWallY = 0, topWallY = 0;
      bool foundLeft = false, foundRight = false;
      bool foundBottom = false, foundTop = false;

      // Peaks (Wände) im Histogramm suchen
      for (int i = 0; i < NUM_BINS; i++) {
        if (histX[i] >= PEAK_THRESHOLD) { leftWallX = (i * BIN_SIZE_MM) - HISTOGRAM_HALF_RANGE; foundLeft = true; break; }
      }
      for (int i = NUM_BINS - 1; i >= 0; i--) {
        if (histX[i] >= PEAK_THRESHOLD) { rightWallX = (i * BIN_SIZE_MM) - HISTOGRAM_HALF_RANGE; foundRight = true; break; }
      }
      for (int i = 0; i < NUM_BINS; i++) {
        if (histY[i] >= PEAK_THRESHOLD) { bottomWallY = (i * BIN_SIZE_MM) - HISTOGRAM_HALF_RANGE; foundBottom = true; break; }
      }
      for (int i = NUM_BINS - 1; i >= 0; i--) {
        if (histY[i] >= PEAK_THRESHOLD) { topWallY = (i * BIN_SIZE_MM) - HISTOGRAM_HALF_RANGE; foundTop = true; break; }
      }

      // X-Achse korrigieren
      if (foundLeft && foundRight) {
          float rawX = -(rightWallX + leftWallX) / 2.0f;
          float errX = rawX - est_px;
          
          est_px += errX * 0.4f;   // Position sanft nachziehen
          est_vx += errX * 2.0f;   // Geschwindigkeits-Drift fixen
      }

      // Y-Achse korrigieren
      if (foundTop && foundBottom) {
          float rawY = -(topWallY + bottomWallY) / 2.0f;
          float errY = rawY - est_py;
          
          est_py += errY * 0.4f;
          est_vy += errY * 2.0f;
      }
  }

  // ==========================================================
  // 4. Finale Feldgrenzen und Ausgabe
  // ==========================================================
  float fieldHalfX = FIELD_X_MM / 2.0f;
  float fieldHalfY = FIELD_Y_MM / 2.0f;

  Player.x = constrain(est_px, -fieldHalfX, fieldHalfX);
  Player.y = constrain(est_py, -fieldHalfY, fieldHalfY);
}

void lidaar() {
  lidarsensor.loop();
  processLidarData();
}