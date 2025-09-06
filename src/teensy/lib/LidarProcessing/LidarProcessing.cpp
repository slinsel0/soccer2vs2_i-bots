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
extern GyroSystem gyros;

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
  resetSimilarPoints();          // Histogramme auf 0 setzen

  // --- 1) Histogramme füllen (Schrittweite 10 mm = 1 cm) ---------------------
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

  // --- 2) Äußerste belegte X- und Y-Bins suchen ------------------------------
  highestX = lowestX = highestY = lowestY = 0;

  for (int i = FIELD_X_CM; i >= 0; --i) {            // +X-Wand
    if (similarPointsX[i][0] > 1) { highestX =  i; break; }
  }
  for (int i = FIELD_X_CM; i >= 0; --i) {            // –X-Wand
    if (similarPointsX[i][1] > 1) { lowestX  = -i; break; }
  }
  for (int i = FIELD_Y_CM; i >= 0; --i) {            // +Y-Wand
    if (similarPointsY[i][0] > 1) { highestY =  i; break; }
  }
  for (int i = FIELD_Y_CM; i >= 0; --i) {            // –Y-Wand
    if (similarPointsY[i][1] > 1) { lowestY  = -i; break; }
  }

  // --- 3) Roboter­mittelpunkt berechnen --------------------------------------
  Player.x = (highestX + lowestX) * 0.5f;   // cm-Koordinaten
  Player.y = (highestY + lowestY) * 0.5f;
}
void clusterPoints() {
  resetSimilarPoints();
  int xLimit = FIELD_X_CM;
  int yLimit = FIELD_Y_CM;

  // Cluster X-Werte
  for (int i = 0; i < 360; i++) {
    int Xdiv10 = points360[i].x / 10;
    if (abs(Xdiv10) < xLimit) {
      if (Xdiv10 >= 0)
        similarPointsX[Xdiv10][0]++;
      else
        similarPointsX[abs(Xdiv10)][1]++;
    }
  }
  mostCommonX = 0;
  mostCommonXValue = 0;
  for (int i = 0; i < xLimit + 1; i++) {
    if (similarPointsX[i][0] > mostCommonXValue) {
      mostCommonXValue = similarPointsX[i][0];
      mostCommonX = i;
    }
    if (similarPointsX[i][1] > mostCommonXValue) {
      mostCommonXValue = similarPointsX[i][1];
      mostCommonX = -i;
    }
  }

  // Cluster Y-Werte
  for (int i = 0; i < 360; i++) {
    int Ydiv10 = points360[i].y / 10;
    if (abs(Ydiv10) < yLimit) {
      if (Ydiv10 >= 0)
        similarPointsY[Ydiv10][0]++;
      else
        similarPointsY[abs(Ydiv10)][1]++;
    }
  }
  mostCommonY = 0;
  mostCommonYValue = 0;
  for (int i = 0; i < yLimit + 1; i++) {
    if (similarPointsY[i][0] > mostCommonYValue) {
      mostCommonYValue = similarPointsY[i][0];
      mostCommonY = i;
    }
    if (similarPointsY[i][1] > mostCommonYValue) {
      mostCommonYValue = similarPointsY[i][1];
      mostCommonY = -i;
    }
  }

  // Markiere Punkte nahe an den Wänden
  for (int i = 0; i < 360; i++) {
    if ((points360[i].x <= (highestX + 1) * 10 && points360[i].x > highestX * 10 - wallPuffer) ||
        (points360[i].x >= (lowestX - 1) * 10 && points360[i].x < lowestX * 10 + wallPuffer)) {
      points360[i].status = 1;
    }
    if ((points360[i].y <= (highestY + 1) * 10 && points360[i].y > highestY * 10 - wallPuffer) ||
        (points360[i].y >= (lowestY - 1) * 10 && points360[i].y < lowestY * 10 + wallPuffer)) {
      points360[i].status = 1;
    }
  }

  noClusterPoints = 0;
  for (int i = 0; i < 360; i++) {
    if (points360[i].status != 1 && points360[i].distance > selfSize)
      noClusterPoints++;
  }

  // Objekterkennung: Gruppiere benachbarte Punkte
  objects.clear();
  for (int i = 0; i < 360; i++) {
    if (points360[i].status != 1 && points360[i].distance > selfSize) {
      int startPoint = i;
      int endPoint = i;
      for (int j = i + 1; j < 360; j++) {
        if (points360[j].status != 1 && points360[j].distance > selfSize)
          endPoint = j;
        else {
          i = j;
          break;
        }
      }
      if (endPoint - startPoint > 2) {
        int midPoint = (startPoint + endPoint) / 2;
        Vec2 midpointCoords;
        midpointCoords.x = -points360[midPoint].x;
        midpointCoords.y = -points360[midPoint].y;
        int sqDist = midpointCoords.x * midpointCoords.x + midpointCoords.y * midpointCoords.y;
        if (sqDist > 180 * 180)
          objects.push_back(midpointCoords);
      }
    }
  }
}

void setBotX() {
  int xLimit = FIELD_X_CM;
  for (int i = 0; i < xLimit + 1; i++) {
    if (similarPointsX[xLimit - i][0] > 1) {
      highestX = xLimit - i;
      break;
    }
  }
  for (int i = 0; i < xLimit + 1; i++) {
    if (similarPointsX[xLimit - i][1] > 1) {
      lowestX = -(xLimit - i);
      break;
    }
  }
  Player.x = (highestX + lowestX) / 2.0f;
}

void setBotY() {
  int yLimit = FIELD_Y_CM;
  for (int i = 0; i < yLimit + 1; i++) {
    if (similarPointsY[yLimit - i][0] > 1) {
      highestY = yLimit - i;
      break;
    }
  }
  for (int i = 0; i < yLimit + 1; i++) {
    if (similarPointsY[yLimit - i][1] > 1) {
      lowestY = -(yLimit - i);
      break;
    }
  }
  Player.y = (highestY + lowestY) / 2.0f;
}

void calculateBotPos() {
  setBotX();
  setBotY();
}

void lidaar() {
  lidarsensor.loop();
  double angleRadians = gyros.getAngleRadians();
  double cosTheta = cos(angleRadians);
  double sinTheta = sin(angleRadians);

  // Gehe über alle LiDAR-Messungen
  for (int i = 0; i < POINTCLOUD_SIZE; i++) {
    double x = lidarsensor.lidar_points[i].y; // x und y getauscht
    double y = lidarsensor.lidar_points[i].x;
    if (x == 0 && y == 0)
      continue;

    // Rotationsblock (1:1 übernehmen)
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
  calculateBotPos();

}

void LidarBegin()
{
  lidarsensor.begin(&Serial1);
}
