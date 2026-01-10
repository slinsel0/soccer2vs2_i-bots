#include "LidarProcessing.h"
#include "math.h"
#include <vector>
#include "GyroSystem.h"
#include "LD19.h"  // Damit kennen wir POINTCLOUD_SIZE (sofern du es als Makro definierst) und die Klasse LD19





// --- Globale Variablen ---

Vec2 Player;  // Roboterposition

LidarPoint points360[360];

// Für Clustering (Anzahl der LiDAR-Punkte in einem bestimmten Bereich in cm)
int similarPointsX[FIELD_X_CM + 1][2];  // [0] für positive, [1] für negative Werte
int similarPointsY[FIELD_Y_CM + 1][2];

int mostCommonX = 0;
int mostCommonXValue = 0;
int mostCommonY = 0;
int mostCommonYValue = 0;

int highestX = 0;
int lowestX = 0;
int highestY = 0;
int lowestY = 0;

int wallPuffer = 10;    // Puffer (in mm) für Wandnähe
int noClusterPoints = 0;
int selfSize = 180;     // Eigengröße des Roboters (mm)


// Vektoren für erkannte Objekte/Gegner
std::vector<Vec2> objects;
std::vector<Vec2E> Enemys;
std::vector<Vec2> EnemysFiltered;



// --- LiDAR-Sensor ---
// Erzeuge eine globale Instanz deines LD19-Sensors
 LD19 lidarsensor;
extern GyroSystem gyro;

// --- Funktionen ---

Vec2 Vec2EToVec2(const Vec2E &vec) {
  Vec2 result;
  result.x = vec.x;
  result.y = vec.y;
  return result;
}

bool IsPointInCircle(Vec2 point, Vec2 center, float radius) {
  float dx = point.x - center.x;
  float dy = point.y - center.y;
  return (dx * dx + dy * dy) <= (radius * radius);
}

void resetPoint(int i) {
  points360[i].distance  = 0;
  points360[i].intensity = 0;
  points360[i].age       = 0;
  points360[i].x         = 0;
  points360[i].y         = 0;
  points360[i].status    = 0;
}

void countCycle() {
  for (int i = 0; i < 360; i++) {
    points360[i].age++;
  }
}

void filterPoints() {
  for (int i = 0; i < 360; i++) {
    // Beispielhafter Alterstest (legalage = 10)
    if (points360[i].age > 10) {
      resetPoint(i);
      continue;
    }
    if (abs(points360[i].x) > FIELD_X_MM || abs(points360[i].y) > FIELD_Y_MM) {
      points360[i].status = 999;
    }
    if ((points360[i].intensity < 100 && points360[i].intensity != 0) ||
        points360[i].distance > sqrt(FIELD_X_MM * FIELD_X_MM + FIELD_Y_MM * FIELD_Y_MM)) {
      points360[i].status = 999;
    }
  }
}

void resetSimilarPoints() {
  int xSize = FIELD_X_CM + 1;
  int ySize = FIELD_Y_CM + 1;
  for (int i = 0; i < xSize; i++) {
    similarPointsX[i][0] = 0;
    similarPointsX[i][1] = 0;
  }
  for (int i = 0; i < ySize; i++) {
    similarPointsY[i][0] = 0;
    similarPointsY[i][1] = 0;
  }
}
void clusterForPos() {
  resetSimilarPoints();          

  for (int i = 0; i < 360; ++i) {
    int xBin = points360[i].x / 10;
    int yBin = points360[i].y / 10;

    if (abs(xBin) <= FIELD_X_CM) {
      (xBin >= 0 ? similarPointsX[xBin][0] : similarPointsX[-xBin][1])++;
    }
    if (abs(yBin) <= FIELD_Y_CM) {
      (yBin >= 0 ? similarPointsY[yBin][0] : similarPointsY[-yBin][1])++;
    }
  }

  // --- 2) Äußerste  X und Y suchen ------------------------------
  // --- 2) Robuster Position Calculation -----------------------
  
  // Static state for "memory"
  static float lastPlayerX = 0.0f;
  static bool  firstRunX   = true;

  static float lastPlayerY = 0.0f;
  static bool  firstRunY   = true;

  // --- X-AXIS ---
  int bestX = 0; // The chosen X position (in cm, relative to center)
  
  // Find candidates
  int candidateHighX = -9999;
  for (int i = FIELD_X_CM; i >= 0; --i) { 
    if (similarPointsX[i][0] > 1) { candidateHighX = i; break; }
  }
  
  int candidateLowX = -9999;
  for (int i = FIELD_X_CM; i >= 0; --i) {
    if (similarPointsX[i][1] > 1) { candidateLowX = -i; break; }
  }

  // Logic: 
  // 1. If we see BOTH walls and distance is correct approx FIELD_X_CM*2, trust them.
  // 2. If valid width, use average.
  // 3. If invalid width (one missing, or obscured), check which one is closer to last known position.
  
  // Check if we found walls at all
  bool foundHighX = (candidateHighX != -9999);
  bool foundLowX  = (candidateLowX  != -9999);

  if (firstRunX) {
      // First run: If we see both, great. If only one, take it.
      if (foundHighX && foundLowX) {
          Player.x = (candidateHighX + candidateLowX) * 0.5f * 10.0f;
      } else if (foundHighX) {
          Player.x = (candidateHighX - FIELD_X_CM) * 10.0f; // Assuming we are at least somewhat valid relative to that wall? No, better: assume center off that wall
           // Actually, without previous info, hard to guess. Let's just assume we are relative to that wall as if it was correct 
           // but mapped to global? 
           // If we only see one wall, we know coordinate = WallPos - FieldWidth/2? No.
           // Global 0 is center. Wall is at +FIELD_X_CM. 
           // So if we see wall at dist d, pos = FIELD_X_CM - d? In our bins, candidateHighX IS the coordinate.
           // Our bins are "xBin". xBin = x / 10. 
           // The scanner measures points relative to ROBOT. 
           // So `points360[i].x` is x-coord relative to robot.
           // `similarPointsX` bins are based on these relative coords.
           // Wait, `points360[i].x` is relative to robot.
           // If robot is at 0, Wall is at +90cm. Scanner sees point at +90. Bin +90.
           // If robot is at +50, Wall is at +40 relative. Scanner sees +40. Bin +40.
           // So `candidateHighX` is "Distance to Positive Wall".
           // Robot Position = FieldMaxX - CandidateHighX (roughly).
           // Wait, Field Max X is 90 (FIELD_X_CM).
           // If robot is at 0, Wall is 90 away? No, `points360[i].x` is coordinates in ROBOT frame.
           // If robot is facing 0 deg (towards +X), points on +X wall have x > 0.
           // So candidateHighX is the measured relative distance to the front wall.
           // Global X = (FIELD_X_CM) - candidateHighX? 
           // Let's check: 
           // Robot at 0. Front Wall at +91cm global. Relative x = +91. candidateHighX = 91.
           // Global X = 91 - 91 = 0. Correct.
           // Robot at +50. Front Wall at +91 global. Relative x = +41. candidateHighX = 41.
           // Global X = 91 - 41 = 50. Correct.
           // Back Wall at -91 global. Relative x = -141. candidateLowX = -141. (abs is 141, but we stored signed index?)
           // similarPointsX logic: `(xBin >= 0 ? similarPointsX[xBin][0] : similarPointsX[-xBin][1])++;`
           // And candidateLowX retrieval: `lowestX = -i`. So it is signed.
           // Global X from low wall: -91 - candidateLowX?
           // Robot at 0. Back Wall -91. Relative -91. candidateLowX = -91.
           // Global X = -91 - (-91) = 0.
           // Robot at +50. Back Wall -91. Relative -141. candidateLowX = -141.
           // Global X = -91 - (-141) = 50. Correct.
           
           // So:
           // PosFromHigh = FIELD_X_CM - candidateHighX
           // PosFromLow  = -FIELD_X_CM - candidateLowX
           
           // Wait, FIELD_X_CM is usually *Half* width? Or full? 
           // `#define FIELD_X_MM 1820` -> 182cm.
           // `#define FIELD_X_CM (FIELD_X_MM / 10)` -> 182.
           // If 0 is center, then walls are at +/- 91. 
           // `abs(xBin) <= FIELD_X_CM` -> checks up to 182? 
           // This implies FIELD_X_CM is FULL width or half? 
           // The check `abs` suggests it is treated as a radius bound if 0 is center? 
           // But `FIELD_X_MM` is 1820. Standard field is 182x243? Or 182 total?
           // Usually RCJ Soccer fields are ~182x243 TOTAL. 
           // So center to wall is 91cm.
           // If FIELD_X_CM is 182, then `abs(xBin) <= 182` means we look for points up to 1.8m away.
           // If robot is at -80, wall is at +170? Yes.
           // So FIELD_X_CM is likely acting as a "Max Sensor Range of interest" or "Max Field Dimension".
           // But for the position calc, we need TRUE WALL POSITION.
           // Let's assume Global Walls are at +/- (FIELD_X_CM / 2).
           
           float halfFieldX = FIELD_X_CM / 2.0f; // 91
           
           if(foundHighX && foundLowX) {
               float p1 = halfFieldX - candidateHighX;
               float p2 = -halfFieldX - candidateLowX;
               Player.x = (p1 + p2) * 0.5f * 10.0f;
           } else {
               Player.x = 0; // Fallback
           }
           lastPlayerX = Player.x;
           firstRunX = false;
      }
  } else {
      // Normal run - use lastPlayerX to validate
      float halfFieldX = FIELD_X_CM / 2.0f;
      
      float valHigh = foundHighX ? (halfFieldX - candidateHighX) : -99999;
      float valLow  = foundLowX  ? (-halfFieldX - candidateLowX) : -99999;

      // Check consensus
      bool validHigh = foundHighX;
      bool validLow  = foundLowX;
      
      // Filter jumps: If new pos is wildly different from last (> 30cm), might be opponent
      // But if BOTH agree, it's probably a fast move or we were wrong before.
      // If only one exists, we MUST check dist.
      
      if(foundHighX && foundLowX) {
         // Check if they agree with each other (Width check)
         // Expected diff: candidateHighX - candidateLowX should be roughly FIELD_X_CM (182)
         // (halfFieldX - p) - (-halfFieldX - p) -> NO. 
         // Relative distance between walls = candidateHighX - candidateLowX.
         // correct: 182.
         int width = candidateHighX - candidateLowX; // e.g. 91 - (-91) = 182
         if (abs(width - FIELD_X_CM) < 15) { // 15cm tolerance
             // Consistent. Trust average.
             Player.x = (valHigh + valLow) * 0.5f * 10.0f;
         } else {
             // Inconsistent width. One is wrong (opponent).
             // Which one is closer to last known?
             float distHigh = abs(valHigh * 10.0f - lastPlayerX);
             float distLow  = abs(valLow * 10.0f - lastPlayerX);
             if (distHigh < distLow) {
                 Player.x = valHigh * 10.0f;
             } else {
                 Player.x = valLow * 10.0f;
             }
         }
      } else if (foundHighX) {
          // Only high. Is it valid?
          // If jump is too large, ignore? But if we ignore, we never update.
          // Better: limit slew rate or check if "jump" is impossible.
          // For now: Just trust it? No, that causes the overshooting if opponent blocks.
          // If validation against memory fails, keep old? 
          // Simple check: If valHigh is within 40cm of last, take it. Else ignore (or creep).
          if (abs(valHigh * 10.0f - lastPlayerX) < 400) { // 40cm
              Player.x = valHigh * 10.0f;
          } else {
              // suspect. Keep last or move slightly?
              // Moving slightly prevents getting stuck.
              Player.x = lastPlayerX * 0.9f + (valHigh * 10.0f) * 0.1f; 
          }
      } else if (foundLowX) {
          if (abs(valLow * 10.0f - lastPlayerX) < 400) {
              Player.x = valLow * 10.0f;
          } else {
              Player.x = lastPlayerX * 0.9f + (valLow * 10.0f) * 0.1f;
          }
      } else {
          // No walls. Keep last or decay to 0?
          // Keep last is safer for short blind spots.
      }
      
      lastPlayerX = Player.x;
  }
 

  // --- Y-AXIS (Same Logic) ---
  int candidateHighY = -9999;
  for (int i = FIELD_Y_CM; i >= 0; --i) {
    if (similarPointsY[i][0] > 1) { candidateHighY = i; break; }
  }
  
  int candidateLowY = -9999;
  for (int i = FIELD_Y_CM; i >= 0; --i) {
    if (similarPointsY[i][1] > 1) { candidateLowY = -i; break; }
  }
  
  bool foundHighY = (candidateHighY != -9999);
  bool foundLowY  = (candidateLowY  != -9999);
  float halfFieldY = FIELD_Y_CM / 2.0f;

  if (firstRunY) {
      if (foundHighY && foundLowY) {
          float p1 = halfFieldY - candidateHighY;
          float p2 = -halfFieldY - candidateLowY;
          Player.y = (p1 + p2) * 0.5f * 10.0f;
      } else { // approximate or 0
          Player.y = 0; 
      }
      lastPlayerY = Player.y;
      firstRunY = false;
  } else {
      float valHigh = foundHighY ? (halfFieldY - candidateHighY) : -99999;
      float valLow  = foundLowY  ? (-halfFieldY - candidateLowY) : -99999;

      if(foundHighY && foundLowY) {
         int width = candidateHighY - candidateLowY;
         if (abs(width - FIELD_Y_CM) < 15) {
             Player.y = (valHigh + valLow) * 0.5f * 10.0f;
         } else {
             float distHigh = abs(valHigh * 10.0f - lastPlayerY);
             float distLow  = abs(valLow * 10.0f - lastPlayerY);
             if (distHigh < distLow) Player.y = valHigh * 10.0f;
             else Player.y = valLow * 10.0f;
         }
      } else if (foundHighY) {
          if (abs(valHigh * 10.0f - lastPlayerY) < 400) Player.y = valHigh * 10.0f;
          else Player.y = lastPlayerY * 0.9f + (valHigh * 10.0f) * 0.1f; 
      } else if (foundLowY) {
          if (abs(valLow * 10.0f - lastPlayerY) < 400) Player.y = valLow * 10.0f;
          else Player.y = lastPlayerY * 0.9f + (valLow * 10.0f) * 0.1f;
      }
      lastPlayerY = Player.y;
  }
}

void lidaar() {
  lidarsensor.loop();
  double angleRadians = gyro.getAngleRadians();
  double cosTheta = cos(angleRadians);
  double sinTheta = sin(angleRadians);

  // Gehe über alle LiDAR-Messungen
  for (int i = 0; i < POINTCLOUD_SIZE; i++) {
    double x = lidarsensor.lidar_points[i].y; // x und y getauscht
    double y = lidarsensor.lidar_points[i].x;
    if (x == 0 && y == 0)
      continue;

    double xRotated = x * cosTheta - y * sinTheta;
    double yRotated = x * sinTheta + y * cosTheta;
    double xAdjusted = yRotated;
    double yAdjusted = xRotated * (-1);

    int angleDeg = static_cast<int>(atan2(yAdjusted, xAdjusted) * 180.0 / PI);
    if (angleDeg < 0)
      angleDeg += 360;
    int index = angleDeg % 360;
    int distance = static_cast<int>(sqrt(xAdjusted * xAdjusted + yAdjusted * yAdjusted));

    points360[index].distance  = distance;
    points360[index].intensity = lidarsensor.lidar_points[i].intensity;
    points360[index].age       = 0;
    points360[index].x         = static_cast<int>(xAdjusted);
    points360[index].y         = static_cast<int>(yAdjusted);
    points360[index].status    = 0;
  }

  countCycle();
  filterPoints();
  // clusterPoints();
  clusterForPos();
  // calculateBotPos();

}

void LidarBegin()
{
  lidarsensor.begin(&Serial1);
}
