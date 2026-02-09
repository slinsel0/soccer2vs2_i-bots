#include "outofbounce.h"
#include <algorithm> 
#include <math.h>

// Kleine Hilfsfunktionen für sauberen Code
static inline float sgn(float val) {
  if (val > 0.0f) return 1.0f;
  if (val < 0.0f) return -1.0f;
  return 0.0f;
}

static inline float clampf(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

void applyFieldBounds(Vec2& cmd,
                      float px, float py,
                      const BoundsConfig& cfg,
                      const Vec2& ballLocal,
                      const BoundsExtras& extras)
{
  // ----------------------------------------------------------
  // 1. Escape-Logik (bleibt wie gehabt)
  // ----------------------------------------------------------
  bool escapeX = false;
  bool escapeY = false;

  if (extras.enableDirectionalEscape) {
    // Wenn Ball deutlich in der anderen Hälfte liegt -> Y-Grenze lockern
    if ( (py > 0.0f && ballLocal.y < -extras.yEscapeThresh) ||
         (py < 0.0f && ballLocal.y >  +extras.yEscapeThresh) ) {
      escapeY = true;
    }
    // Wenn Ball deutlich auf der anderen Seite liegt -> X-Grenze lockern
    if ( (px >  (cfg.xLimit - cfg.softMargin) && ballLocal.x < -extras.xEscapeThresh) ||
         (px < -(cfg.xLimit - cfg.softMargin) && ballLocal.x >  +extras.xEscapeThresh) ) {
      escapeX = true;
    }
  }

  // ----------------------------------------------------------
  // 2. Fluide Begrenzungs-Logik (Proportional + Exponentiell)
  // ----------------------------------------------------------
  
  auto applyAxisPhysics = [&](float &velCmd, float pos, float limit, bool isEscape) {
      float absPos = fabsf(pos);
      float signPos = sgn(pos);
      
      // Zonen-Definition
      float startSoft = limit - cfg.softMargin; // Hier beginnt das Bremsen
      float startHard = limit - cfg.hardMargin; // Hier ist Schluss (Wand)

      // Wenn wir noch weit weg sind oder "Escape" aktiv ist -> Abbruch
      if (absPos < startSoft || isEscape) return;

      // Prüfen, ob der Roboter versucht, RAUS zu fahren
      // (Geschwindigkeit hat gleiches Vorzeichen wie Position)
      bool movingOut = (velCmd * signPos > 0.0f);

      // --- A: Soft Zone (Progressives Bremsen) ---
      if (movingOut) {
          float zoneWidth = cfg.softMargin - cfg.hardMargin;
          if (zoneWidth < 0.001f) zoneWidth = 0.001f; // Div/0 Schutz

          float penetration = (absPos - startSoft) / zoneWidth;
          penetration = clampf(penetration, 0.0f, 1.0f);

          // Die maximal erlaubte Geschwindigkeit sinkt linear
          float currentLimit = (1.0f - penetration) * cfg.maxSoft;

          // Drosseln, falls zu schnell
          if (fabsf(velCmd) > currentLimit) {
              velCmd = signPos * currentLimit;
          }
      }

      // --- B: Hard Zone (Exponenzieller Rückstoß) ---
      // Wenn wir über die Linie rutschen, drückt uns eine "Feder" zurück.
      if (absPos > startHard) {
          float deep = absPos - startHard; // Eindringtiefe in cm
          
          // Exponenzieller Rückstoß: F = k * (e^x - 1)
          // Das fühlt sich bei kleiner Tiefe weich an (x ~= e^x-1) und wird dann brutal hart.
          // Mit kPush = 1.0 hat man bei 4cm Eindringtiefe schon Max-Power.
          float pushForce = cfg.kPush * (expf(deep) - 1.0f);
          
          // Begrenzung der Rückstoßkraft (gegen Oszillation und Übersteuern)
          pushForce = clampf(pushForce, 0.0f, cfg.maxHard);

          // Kraft wirkt immer nach INNEN (entgegen der Position)
          velCmd -= signPos * pushForce;
      }
  };

  // Physik auf beide Achsen anwenden
  applyAxisPhysics(cmd.x, px, cfg.xLimit, escapeX);
  applyAxisPhysics(cmd.y, py, cfg.yLimit, escapeY);
  
  // (Optional) Vektor-Limitierung
  float vSq = cmd.x*cmd.x + cmd.y*cmd.y;
  float absoluteMax = cfg.maxHard * 1.41f + 10.0f; // Toleranz
  if (vSq > absoluteMax * absoluteMax) {
      float scale = absoluteMax / sqrtf(vSq);
      cmd.x *= scale;
      cmd.y *= scale;
  }
}