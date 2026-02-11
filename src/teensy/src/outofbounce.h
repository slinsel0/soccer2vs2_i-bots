#pragma once
#include <math.h>

#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 {
    float x;
    float y;
};
#endif

/*  ═══════════════════════════════════════════════════════════════════
 *  AXIS-LOCK  +  PULL-TO-LINE  +  TOR-ZONEN
 *  ═══════════════════════════════════════════════════════════════════
 *
 *  RCJ Soccer Field 2023  (182 × 243 cm)
 *
 *       ┌──────────┬────────────────┬──────────┐
 *       │  WAND    │   BLUE GOAL    │   WAND   │  Y = +yLimit
 *       │          │    (60 cm)     │          │
 *       ├──────────┘                └──────────┤
 *       │                                      │
 *       │             Spielfeld                │
 *       │            (182 × 243)               │
 *       │                                      │
 *       │              (0,0)                   │
 *       │                ·                     │
 *       │                                      │
 *       │                                      │
 *       │                                      │
 *       ├──────────┐                ┌──────────┤
 *       │  WAND    │  YELLOW GOAL   │   WAND   │  Y = -yLimit
 *       │          │    (60 cm)     │          │
 *       └──────────┴────────────────┴──────────┘
 *     X=-xLimit   X=-30    0    X=+30       X=+xLimit
 *
 *
 *  Regeln:
 *    X-Achse:  Seitenwände – safeLine überall gleich
 *
 *    Y-Achse:  Hängt von X-Position ab!
 *      |x| > goalHalfW  →  Bot steht VOR WAND    → safeLine_Y normal
 *      |x| ≤ goalHalfW  →  Bot steht VOR TOR     → safeLine_Y ENGER
 *                           (sonst fährt er ins Tor rein!)
 *
 *    Axis-Lock + Escape:  wie vorher
 *      Achse draussen + cmd outward  →  LOCK (cmd = Pull)
 *      Achse draussen + cmd inward   →  ESCAPE (cmd + Pull)
 *
 *  ═══════════════════════════════════════════════════════════════════ */

struct BoundsConfig {
  // ── Feldgrenzen (halbe Maße, cm) ──
  float xLimit;           // Seitenwand      (91.0 = 182/2)
  float yLimit;           // Torseite-Wand   (121.5 = 243/2)

  // ── Tor-Geometrie ──
  float goalHalfWidth;    // halbe Torbreite  (30.0 = 60/2)

  // ── Sicherheitsmargen ──
  float safeMarginX;      // Abstand safeLine ↔ Seitenwand
  float safeMarginY;      // Abstand safeLine ↔ Wand (neben Tor)
  float goalSafeMarginY;  // Abstand safeLine ↔ Torlinie (VOR dem Tor)
                          //   Muss GRÖSSER sein als safeMarginY!
                          //   Sonst fährt Bot ins Tor.

  // ── Pull-Regler ──
  float kPull;            // Verstärkung: Pull = kPull × overshoot
  float maxPull;          // Maximale Pull-Geschwindigkeit
};


void applyFieldBounds(Vec2& cmd,
                      float px, float py,
                      const BoundsConfig& cfg);