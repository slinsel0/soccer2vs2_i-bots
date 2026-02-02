#include "outofbounce.h"
#include <algorithm>
#include <math.h>

static inline float sgn(float v) { return (v > 0.0f) - (v < 0.0f); }

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// 0..1 smoothstep, continuous slope at ends
static inline float smooth01(float x) {
  x = clampf(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

void applyFieldBounds(Vec2& cmd,
                      float px, float py,
                      const BoundsConfig& cfg,
                      const Vec2& ballLocal,
                      const BoundsExtras& extras)
{
  // ---- config sanity ----
  float softM = cfg.softMargin;
  float hardM = cfg.hardMargin;
  if (softM < 1.0f) softM = 1.0f;
  if (hardM < 0.0f) hardM = 0.0f;
  if (hardM > softM - 0.5f) hardM = softM - 0.5f;

  // ---- Escape-Heuristik: lockert nur Soft-Zone, nie deaktivieren ----
  bool escapeX = false;
  bool escapeY = false;

  if (extras.enableDirectionalEscape) {
    if ((py > 0.0f && ballLocal.y < -extras.yEscapeThresh) ||
        (py < 0.0f && ballLocal.y > +extras.yEscapeThresh)) {
      escapeY = true;
    }
    if ((px >  (cfg.xLimit - softM) && ballLocal.x < -extras.xEscapeThresh) ||
        (px < -(cfg.xLimit - softM) && ballLocal.x > +extras.xEscapeThresh)) {
      escapeX = true;
    }
  }

  auto applyAxis = [&](float &vCmd, float pos, float limit, bool isEscape) {
    const float absPos  = fabsf(pos);
    const float signPos = sgn(pos);

    // distance to wall (cm): d>0 inside, d=0 at line, d<0 outside
    const float d = limit - absPos;

    // Outside Panic: erzwinge Rückkehr nach innen
    if (d < -extras.outsidePanicCm) {
      if (vCmd * signPos > 0.0f) vCmd = 0.0f;       // outward kill
      const float minIn = cfg.maxHard;              // min return speed
      if (vCmd * signPos > -minIn) vCmd = -signPos * minIn;
      return;
    }

    // Soft-Margin ggf. lockern (Escape), aber Hard bleibt hard
    float softEff = softM;
    if (isEscape) {
      softEff = std::max(hardM + extras.minSoftHardGap, softM * extras.escapeSoftFactor);
    }

    const float startSoft = limit - softEff;
    const float startHard = limit - hardM;

    if (absPos < startSoft) return;

    const bool movingOut = (vCmd * signPos > 0.0f);

    // Soft-Zone: outward-Komponente progressiv dämpfen
    if (movingOut) {
      // x=1 bei Soft-Start, x=0 bei Hard-Start
      const float x = (startHard - absPos) / (startHard - startSoft);
      const float s = smooth01(x);

      // Quadratisch -> weniger Limit-Cycle
      const float vOutMax = cfg.maxSoft * (s * s);
      const float vAbs = fabsf(vCmd);
      if (vAbs > vOutMax) vCmd = signPos * vOutMax;
    }

    // Hard-Zone: definierter Push nach innen
    if (absPos > startHard) {
      const float pen = clampf(absPos - startHard, 0.0f, hardM + extras.outsidePanicCm);
      float push = cfg.kPush * pen * pen;          // stabiler Ramp, kein exp()
      push = clampf(push, 0.0f, cfg.maxHard);
      vCmd -= signPos * push;
    }
  };

  applyAxis(cmd.x, px, cfg.xLimit, escapeX);
  applyAxis(cmd.y, py, cfg.yLimit, escapeY);

  // Globaler Vektor-Limit (verhindert Diagonal-Übertreibung nach Push)
  const float vSq = cmd.x * cmd.x + cmd.y * cmd.y;
  const float absMax = std::max(cfg.maxSoft, cfg.maxHard) * 1.45f;
  if (vSq > absMax * absMax) {
    const float scale = absMax / sqrtf(vSq);
    cmd.x *= scale;
    cmd.y *= scale;
  }
}
