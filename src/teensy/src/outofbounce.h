#pragma once
#include <math.h>

#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 {
    float x;
    float y;
};
#endif

/* ═══════════════════════════════════════════════════════════════════
 * OOB: VECTOR CLAMPING + PULL-BACK + TOR-ZONEN
 * ═══════════════════════════════════════════════════════════════════
 *
 * RCJ Soccer Field 2vs2 Vision (182 × 243 cm)
 * ═══════════════════════════════════════════════════════════════════ */

struct BoundsConfig {
  // ── Feldgrenzen (halbe Maße, in cm) ──
  float xLimit;           // Seitenwand      (91.0)
  float yLimit;           // Torseite-Wand   (121.5)

  // ── Tor-Geometrie ──
  float goalHalfWidth;    // Halbe Torbreite (30.0)

  // ── Sicherheitsmargen ──
  float safeMarginX;      
  float safeMarginY;      
  float goalSafeMarginY;  

  // ── Damping & Return ──
  float dampingMargin;    // Weiches Abbremsen X cm vor der Linie
  float returnKp;         // P-Regler: Wie stark er zurück ins Feld zieht,
                          // falls er über die safeLine geschoben wurde.
};

void applyFieldBounds(Vec2& cmd, float px, float py, const BoundsConfig& cfg);