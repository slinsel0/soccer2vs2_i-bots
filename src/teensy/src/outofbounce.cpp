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

  // --- X-Randbehandlung ---
  float ax = fabsf(px);
  float distX_soft = ax - (cfg.xLimit - cfg.softMargin);
  float distX_hard = ax - (cfg.xLimit - cfg.hardMargin);

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

  // --- Y-Randbehandlung ---
  float ay = fabsf(py);
  float distY_soft = ay - (cfg.yLimit - cfg.softMargin);
  float distY_hard = ay - (cfg.yLimit - cfg.hardMargin);

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

  // --- Gesamtnorm in Grenznähe weich begrenzen ---
  float nearEdge = fmaxf(fmaxf(distX_soft, distY_soft), 0.0f);
  if (nearEdge > 0.0f) {
    float maxNorm = (nearEdge > (cfg.softMargin - cfg.hardMargin))
                      ? (cfg.maxSoft)
                      : (0.5f * (cfg.maxSoft + cfg.maxHard));
    limitVectorNorm(cmd, maxNorm);
  }
}
