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

  // ── Predictive Braking ──
  // Effektiver Damping-Margin = dampingMargin + speedLookAhead * |v_richtung_wand|
  // → Je schneller in Richtung Wand, desto früher startet das Damping.
  float speedLookAhead;   // Sekunden Vorausblick (z.B. 0.15)

  // ── Eigene Penalty Area (an -Y, eigenes Tor) ──
  // PA = { |x| < penaltyHalfWidth, py < -yLimit + penaltyDepth }
  float penaltyHalfWidth; // halbe Breite (X) der Penalty Area
  float penaltyDepth;     // wie tief sie ins Feld reicht (Y, von -yLimit)
  float penaltyExitSpeed; // Aktive Mindest-Auswurf-Geschwindigkeit nach +Y
  float penaltyExitKp;    // tiefenabhängiger Zusatz-Schub (≥ returnKp)
};

void applyFieldBounds(Vec2& cmd, float px, float py, const BoundsConfig& cfg);