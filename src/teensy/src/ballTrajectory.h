#pragma once
#include <math.h>
#include <stdint.h>

#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 { float x; float y; };
#endif

/* ═══════════════════════════════════════════════════════════════════
 * BALL TRAJEKTORIE  (lokal-only Workaround)
 * ═══════════════════════════════════════════════════════════════════
 *
 * Der Pi liefert nur lokale Kamera-Pixel (bx, by). Damit wir trotzdem
 * eine "globale" Trajektorie schätzen können, drehen wir die lokale
 * Beobachtung mit dem Gyro-Heading in einen heading-stabilisierten Frame
 * (de facto Welt-Orientierung, +Y_stable = gegn. Tor). Dort schätzen wir
 * Ball-Velocity per gefiltertem Finite-Difference, prädizieren in die
 * Zukunft und legen einen Anflugpunkt HINTER den zukünftigen Ball entlang
 * der Torrichtung. Anschließend zurück in den Kamera-Frame.
 *
 * Pixel↔cm Kalibration wird NICHT benötigt — alles läuft in Pixeln,
 * weil sowohl Beobachtung als auch PID-Eingang in Pixeln sind.
 * ═══════════════════════════════════════════════════════════════════ */

struct BallTrajectoryConfig {
  float lookAheadMs;        // Prädiktions-Horizont (z.B. 150 ms)
  float emaAlpha;           // Glättung der Velocity (0..1, höher = reaktiver)
  float minDtMs;            // unterer Cutoff für Velocity-Update
  float resetGapMs;         // ab dieser Lücke ohne Update → Reset
  float maxBallSpeed;       // Sättigung gegen Ausreißer (px/s)
  float approachDistance;   // Anflugversatz hinter dem Ball (px, in -Y_stable)
  float lateralLeadGain;    // wie stark seitliches Vorhalten beim Bewegungs-Ball (0..1)
  float alignWidth;         // |bxs| < alignWidth ⇒ "X-aligned" zum Tor
  float alignFrontMin;      // bys > alignFrontMin ⇒ Ball nördlich vom Bot (push-fähig)
};

struct BallTrajectoryState {
  float bxStable, byStable; // letzte Ball-Position im stabilen Frame (px)
  float vxStable, vyStable; // gefilterte Ball-Velocity (px/s)
  uint32_t lastMs;
  uint32_t lastSrcMs;       // letzter Quell-Timestamp (zur Duplikat-Erkennung)
  bool initialized;
};

void btReset(BallTrajectoryState& st);

// Update: nur aufrufen, wenn wirklich ein NEUES Sample vorliegt
// (z.B. nach Empfang eines gültigen Pakets).
void btUpdate(BallTrajectoryState& st,
              const BallTrajectoryConfig& cfg,
              float bxCam, float byCam,
              float headingRad,
              uint32_t nowMs);

// Liefert den Anflug-Zielpunkt im KAMERA-Frame (gleiche Konvention wie
// (bx, by) = (last_vx - CAM_CENTER_X, last_vy - CAM_CENTER_Y)).
// Bei nicht-initialisiertem Tracker wird der aktuelle Ball-Pixel mit
// festem -Y-Versatz zurückgegeben (sicheres Fallback).
Vec2 btApproachTargetCam(const BallTrajectoryState& st,
                         const BallTrajectoryConfig& cfg,
                         float headingRad);

// Optional zur Diagnose: rohe Ball-Velocity im stabilen Frame (px/s).
inline float btSpeedStable(const BallTrajectoryState& st) {
    return sqrtf(st.vxStable * st.vxStable + st.vyStable * st.vyStable);
}
