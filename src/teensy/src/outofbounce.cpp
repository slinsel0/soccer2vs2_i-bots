/*  ═══════════════════════════════════════════════════════════════════════
 *  AXIS-LOCK  +  PULL-TO-LINE  +  TOR-ZONEN
 *  ═══════════════════════════════════════════════════════════════════════
 *
 *  X-Achse:  safeLine ist immer gleich (Seitenwände)
 *
 *  Y-Achse:  safeLine hängt von der X-Position ab!
 *
 *    Bot steht seitlich (|x| > 30)?
 *      → Vor ihm ist WAND  → safeLine_Y = yLimit − safeMarginY     (weit)
 *
 *    Bot steht mittig (|x| ≤ 30)?
 *      → Vor ihm ist TOR   → safeLine_Y = yLimit − goalSafeMarginY (eng!)
 *
 *
 *                   Wand   Tor(60cm)   Wand
 *     Y-Limit ──── ██████ ┃        ┃ ██████
 *                         ┃  GOAL  ┃
 *     safeLine(goal)------┃--------┃------    ← enger!
 *                         ┃        ┃
 *     safeLine(wall)------██████████████------  ← normaler Abstand
 *                              ↑
 *                        goalHalfWidth
 *
 *
 *  Axis-Lock + Escape (pro Achse, unverändert):
 *    |pos| < safeLine?                  → Frei, kein Eingriff
 *    |pos| ≥ safeLine + cmd outward?    → LOCK: cmd = Pull
 *    |pos| ≥ safeLine + cmd inward?     → ESCAPE: cmd bleibt + Pull
 *
 *  ═══════════════════════════════════════════════════════════════════════ */

#include "outofbounce.h"
#include <math.h>

// ─────────────────── Hilfsfunktionen ───────────────────────────

static inline float clampf(float val, float lo, float hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

// ═══════════════════════════════════════════════════════════════

void applyFieldBounds(Vec2& cmd,
                      float px, float py,
                      const BoundsConfig& cfg)
{
  // ── Generische Achsen-Funktion (Lock + Pull + Escape) ─────
  //    safeLine wird von aussen übergeben (pro Achse unterschiedlich)

  auto processAxis = [&](float &velCmd, float pos, float safeLine) {

    float absPos  = fabsf(pos);

    // Innerhalb der safeLine? → Nichts tun
    if (absPos <= safeLine) return;

    float overshoot = absPos - safeLine;
    float signPos   = (pos >= 0.0f) ? 1.0f : -1.0f;

    // Pull-Kraft → immer Richtung Mitte (entgegengesetzt zur Position)
    // pos=+80 -> signPos=+1 -> pullVel muss negativ sein, um zur Mitte (-X/-Y) zu fahren.
    float pullSpeed = cfg.kPull * overshoot;
    pullSpeed = clampf(pullSpeed, 0.0f, cfg.maxPull);
    float pullVel = -signPos * pullSpeed;

    // Fahrvektor zeigt nach AUSSEN?
    // Outward = velCmd hat dasselbe Vorzeichen wie pos (pos>0 und velCmd>0 -> abhauen)
    bool wantsOutward = (velCmd * signPos) > 0.0f;

    if (wantsOutward) {
      // LOCK: Outward-Befehl komplett ersetzen durch Pull zur Mitte
      velCmd = pullVel;
    } 
    // ESCAPE: Wenn der Fahrvektor schon nach INNEN zeigt,
    // machen wir gar nichts. Dadurch kann der Code schneller verlassen werden
    // und der Bot reißt sich direkt von der Line los, sobald er will.
  };


  // ═══════════════════════════════════════════════════════════
  //  1.  X-ACHSE  (Seitenwände – immer gleich)
  // ═══════════════════════════════════════════════════════════

  float safeLineX = cfg.xLimit - cfg.safeMarginX;
  processAxis(cmd.x, px, safeLineX);


  // ═══════════════════════════════════════════════════════════
  //  2.  Y-ACHSE  (dynamisch: Wand oder Tor?)
  // ═══════════════════════════════════════════════════════════
  //
  //  Entscheidung basiert auf X-Position des Bots:
  //    |px| > goalHalfWidth  →  Bot steht VOR WAND
  //    |px| ≤ goalHalfWidth  →  Bot steht VOR TOR-ÖFFNUNG

  float safeLineY;

  if (fabsf(px) <= cfg.goalHalfWidth) {
    // ── Vor dem Tor: safeLine ENGER (damit Bot nicht reinfährt) ──
    safeLineY = cfg.yLimit - cfg.goalSafeMarginY;
  } else {
    // ── Vor der Wand: safeLine normal ────────────────────────────
    safeLineY = cfg.yLimit - cfg.safeMarginY;
  }

  processAxis(cmd.y, py, safeLineY);


  // ═══════════════════════════════════════════════════════════
  //  3.  Absolute Sicherheitsgrenze
  // ═══════════════════════════════════════════════════════════

  constexpr float ABSOLUTE_MAX = 250.0f;
  float vSq = cmd.x * cmd.x + cmd.y * cmd.y;
  if (vSq > ABSOLUTE_MAX * ABSOLUTE_MAX) {
    float scale = ABSOLUTE_MAX / sqrtf(vSq);
    cmd.x *= scale;
    cmd.y *= scale;
  }
}