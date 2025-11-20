#include "outofbounce.h"

static inline float sgn(float v) { return (v >= 0.0f) ? 1.0f : -1.0f; }
static inline float clamp(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}
static inline void limitVectorNorm(Vec2& v, float maxNorm) {
  float n = sqrtf(v.x*v.x + v.y*v.y);
  if (n > maxNorm && n > 1e-6f) {
    float s = maxNorm / n;
    v.x *= s; v.y *= s;
  }
}

// void applyFieldBounds(Vec2& cmd,
//                       float px, float py,
//                       const BoundsConfig& cfg,
//                       const Vec2& ballLocal,
//                       const BoundsExtras& extras)
// {
//   // --- Flags, die Soft-Bremsen gezielt lockern dürfen ---
//   bool escapeX = false;
//   bool escapeY = false;

//   if (extras.enableDirectionalEscape) {
//     // Y-Escape in ABHÄNGIGKEIT der Feldhälfte:
//     // VORN (py>0) + Ball HINTER mir  -> ballLocal.y < -yEscapeThresh
//     // HINTEN (py<0) + Ball VOR mir   -> ballLocal.y > +yEscapeThresh
//     if ( (py > 0.0f && ballLocal.y < -extras.yEscapeThresh) ||
//          (py < 0.0f && ballLocal.y >  +extras.yEscapeThresh) ) {
//       escapeY = true;
//     }

// // X-Escape (Seitenwechsel): Bin ich rechts nah an der Linie und Ball deutlich links?
// if ( (px >  (cfg.xLimit - cfg.softMarginX) && ballLocal.x < -extras.xEscapeThresh) ||
//      (px < -(cfg.xLimit - cfg.softMarginX) && ballLocal.x >  +extras.xEscapeThresh) ) {
//   escapeX = true;
// }

//   }

// // --- X-Randbehandlung ---
// float ax = fabsf(px);
// float distX_soft = ax - (cfg.xLimit - cfg.softMarginX);
// float distX_hard = ax - (cfg.xLimit - cfg.hardMarginX);

// if (distX_hard > 0.0f) {
//   // Harte Zone: proportionaler Rückstoß ins Feld (überstimmt Außenfahrt)
//   float inward = -sgn(px) * (distX_hard + cfg.hardMarginX) * cfg.kPush;
//   cmd.x = clamp(inward, -cfg.maxHard, cfg.maxHard);
// } else if (distX_soft > 0.0f && !escapeX) {
//   // Soft-Zone: Außenfahrt drosseln + leichter Innen-Bias
//   bool goingOut = ((px > 0.0f && cmd.x > 0.0f) || (px < 0.0f && cmd.x < 0.0f));
//   if (goingOut) {
//     cmd.x = clamp(cmd.x, -cfg.maxSoft, cfg.maxSoft);
//     cmd.x += -sgn(px) * (0.2f * distX_soft);
//   }
// }


// // --- Y-Randbehandlung ---
// float ay = fabsf(py);
// float distY_soft = ay - (cfg.yLimit - cfg.softMarginY);
// float distY_hard = ay - (cfg.yLimit - cfg.hardMarginY);

// if (distY_hard > 0.0f) {
//   float inward = -sgn(py) * (distY_hard + cfg.hardMarginY) * cfg.kPush;
//   cmd.y = clamp(inward, -cfg.maxHard, cfg.maxHard);
// } else if (distY_soft > 0.0f && !escapeY) {
//   bool goingOut = ((py > 0.0f && cmd.y > 0.0f) || (py < 0.0f && cmd.y < 0.0f));
//   if (goingOut) {
//     cmd.y = clamp(cmd.y, -cfg.maxSoft, cfg.maxSoft);
//     cmd.y += -sgn(py) * (0.2f * distY_soft);
//   }
// }


// // --- Gesamtnorm in Grenznähe weich begrenzen ---
// float nearEdge = fmaxf(fmaxf(distX_soft, distY_soft), 0.0f);
// if (nearEdge > 0.0f) {
//   // Nutze die "engste" (konservativste) Differenz Soft/Hard über beide Achsen
//   float softMinusHardX = cfg.softMarginX - cfg.hardMarginX;
//   float softMinusHardY = cfg.softMarginY - cfg.hardMarginY;
//   float softMinusHard  = fminf(softMinusHardX, softMinusHardY);

//   float maxNorm = (nearEdge > softMinusHard)
//                     ? (cfg.maxSoft)
//                     : (0.5f * (cfg.maxSoft + cfg.maxHard));
//   limitVectorNorm(cmd, maxNorm);
// }
// }


void applyFieldBounds(Vec2& cmd,
                      float px, float py,
                      const BoundsConfig& cfg,
                      const Vec2& ballLocal,
                      const BoundsExtras& extras)
{
  // --- Flags, die Soft-Bremsen gezielt lockern dürfen ---
  bool escapeX = false;
  bool escapeY = false;

  if (extras.enableDirectionalEscape) {
    // Y-Escape in ABHÄNGIGKEIT der Feldhälfte:
    // VORN (py>0) + Ball HINTER mir  -> ballLocal.y < -yEscapeThresh
    // HINTEN (py<0) + Ball VOR mir   -> ballLocal.y > +yEscapeThresh
    if ( (py > 0.0f && ballLocal.y < -extras.yEscapeThresh) ||
         (py < 0.0f && ballLocal.y >  +extras.yEscapeThresh) ) {
      escapeY = true;
    }

    // X-Escape (Seitenwechsel): Bin ich rechts nah an der Linie und Ball deutlich links?
    if ( (px >  (cfg.xLimit - cfg.softMargin) && ballLocal.x < -extras.xEscapeThresh) ||
         (px < -(cfg.xLimit - cfg.softMargin) && ballLocal.x >  +extras.xEscapeThresh) ) {
      escapeX = true;
    }
  }

  // ------------------------------------------------------------
  // NEU: Fahrvektor merken + Abstände zu den Rändern vorberechnen
  // ------------------------------------------------------------
  const float rawX = cmd.x;   // ursprünglicher Fahrbefehl X
  const float rawY = cmd.y;   // ursprünglicher Fahrbefehl Y

  float ax = fabsf(px);
  float ay = fabsf(py);

  float distX_soft = ax - (cfg.xLimit - cfg.softMargin);
  float distX_hard = ax - (cfg.xLimit - cfg.hardMargin);
  float distY_soft = ay - (cfg.yLimit - cfg.softMargin);
  float distY_hard = ay - (cfg.yLimit - cfg.hardMargin);

  bool lockX = false;  // NEU: X-Achse gelockt?
  bool lockY = false;  // NEU: Y-Achse gelockt?

  // In Ecken nur eine Achse locken: die "stärker" verletzte
  const bool preferX = distX_hard >= distY_hard;

  // Wie weit in die Hard-Zone hinein wir "an der Linie kleben" (0..1)
  constexpr float kLockFrac = 0.30f;

  // ------------------------------------------------------------
  // NEU: X-Lock in Hard-Zone
  // ------------------------------------------------------------
  if (distX_hard > 0.0f && preferX) {
    // Fahrvektor geht wirklich in Richtung Linie?
    const bool drivingIntoX =
        (px > 0.0f && rawX > 0.0f) ||
        (px < 0.0f && rawX < 0.0f);

    if (drivingIntoX && !escapeX) {
      // Ziel: ein Stück innerhalb der Hard-Zone an der Linie bleiben
      const float edgeX  = sgn(px) * (cfg.xLimit - kLockFrac * cfg.hardMargin);
      const float dxEdge = edgeX - px;

      if (fabsf(dxEdge) > 1.0f) {
        // Noch nicht auf der Linie -> aktiv hinfahren
        cmd.x = clamp(dxEdge * cfg.kPush, -cfg.maxHard, cfg.maxHard);
      } else {
        // Linie erreicht -> X-Achse sperren
        cmd.x = 0.0f;
      }
      lockX = true;
    }
  }

  // ------------------------------------------------------------
  // NEU: Y-Lock in Hard-Zone
  // ------------------------------------------------------------
  if (distY_hard > 0.0f && !preferX) {
    const bool drivingIntoY =
        (py > 0.0f && rawY > 0.0f) ||
        (py < 0.0f && rawY < 0.0f);

    if (drivingIntoY && !escapeY) {
      const float edgeY  = sgn(py) * (cfg.yLimit - kLockFrac * cfg.hardMargin);
      const float dyEdge = edgeY - py;

      if (fabsf(dyEdge) > 1.0f) {
        cmd.y = clamp(dyEdge * cfg.kPush, -cfg.maxHard, cfg.maxHard);
      } else {
        cmd.y = 0.0f;
      }
      lockY = true;
    }
  }

  // ------------------------------------------------------------
  // X-Randbehandlung (nur wenn nicht gelockt)
  // ------------------------------------------------------------
  if (!lockX) {
    if (distX_hard > 0.0f) {
      // Harte Zone: proportionaler Rückstoß ins Feld (überstimmt Außenfahrt)
      float inward = -sgn(px) * (distX_hard + cfg.hardMargin) * cfg.kPush;
      cmd.x = clamp(inward, -cfg.maxHard, cfg.maxHard);
    } else if (distX_soft > 0.0f && !escapeX) {
      // Soft-Zone: Außenfahrt drosseln + leichter Innen-Bias
      bool goingOut = ((px > 0.0f && cmd.x > 0.0f) || (px < 0.0f && cmd.x < 0.0f));
      if (goingOut) {
        cmd.x = clamp(cmd.x, -cfg.maxSoft, cfg.maxSoft);
        cmd.x += -sgn(px) * (0.2f * distX_soft);
      }
    }
  }

  // ------------------------------------------------------------
  // Y-Randbehandlung (nur wenn nicht gelockt)
  // ------------------------------------------------------------
  if (!lockY) {
    float ay2 = ay;             // ay ist oben schon berechnet, nur zur Lesbarkeit
    (void)ay2;                  // (um "unused" zu vermeiden, falls du später änderst)

    if (distY_hard > 0.0f) {
      float inward = -sgn(py) * (distY_hard + cfg.hardMargin) * cfg.kPush;
      cmd.y = clamp(inward, -cfg.maxHard, cfg.maxHard);
    } else if (distY_soft > 0.0f && !escapeY) {
      bool goingOut = ((py > 0.0f && cmd.y > 0.0f) || (py < 0.0f && cmd.y < 0.0f));
      if (goingOut) {
        cmd.y = clamp(cmd.y, -cfg.maxSoft, cfg.maxSoft);
        cmd.y += -sgn(py) * (0.2f * distY_soft);
      }
    }
  }

  // --- Gesamtnorm in Grenznähe weich begrenzen ---
  float nearEdge = fmaxf(fmaxf(distX_soft, distY_soft), 0.0f);
  if (nearEdge > 0.0f) {
    float maxNorm = (nearEdge > (cfg.softMargin - cfg.hardMargin))
                      ? (cfg.maxSoft)
                      : (0.5f * (cfg.maxSoft + cfg.maxHard));
    limitVectorNorm(cmd, maxNorm);
  }
}
