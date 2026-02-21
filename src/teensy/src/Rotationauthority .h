#pragma once
/*  ═══════════════════════════════════════════════════════════════════
 *  RotationAuthority  –  Fuzzy Heading-vs-Translation Priorisierung
 *  ═══════════════════════════════════════════════════════════════════
 *
 *  Problem:
 *    rotCmd (Gyro-PID) wird immer gleich stark auf die Motoren addiert,
 *    egal ob der Roboter steht oder mit Vollgas fährt.
 *    → Bei schneller Fahrt: unnötiges Spinnen wegen kleiner Heading-Fehler
 *    → Im Stand: zu langsame Korrektur bei großen Abweichungen
 *
 *  Lösung:
 *    Fuzzy-artige Skalierung der Rotationsautorität anhand von:
 *      1) |heading_error|     (wie schlimm ist die Abweichung?)
 *      2) translation_speed   (wie schnell fährt der Roboter?)
 *
 *  Regeltabelle:
 *    ┌─────────────────┬───────────┬───────────┬───────────┐
 *    │ heading_error → │   SMALL   │  MEDIUM   │    BIG    │
 *    │ speed ↓         │  (<8°)    │  (8–25°)  │  (>25°)   │
 *    ├─────────────────┼───────────┼───────────┼───────────┤
 *    │ STOPPED (<20)   │   0.4     │   0.8     │   1.0     │
 *    │ SLOW    (20-60) │   0.3     │   0.6     │   0.9     │
 *    │ FAST    (>60)   │   0.15    │   0.4     │   0.8     │
 *    └─────────────────┴───────────┴───────────┴───────────┘
 *
 *    Werte dazwischen werden durch die Membership-Funktionen
 *    weich interpoliert → kein harter Sprung.
 *
 *  Benutzung:
 *    #include "RotationAuthority.h"
 *    RotationAuthority rotAuth;           // Default-Params
 *    ...
 *    float scaled = rotAuth.apply(rotCmd, headingError, translationSpeed);
 *    Drive.calcDrive(driveCmd.x, -driveCmd.y, -scaled);
 *
 *  ═══════════════════════════════════════════════════════════════════ */

#include <math.h>

// ─────────────── Tuning-Struktur (alles an einer Stelle) ─────────
struct RotAuthConfig {
  // ── Heading-Error Grenzen (Grad) ──
  float errSmallMax    =  8.0f;   // unter 8° = "klein"
  float errMediumCenter= 16.0f;   // Zentrum der "mittel"-Glocke
  float errMediumWidth = 15.0f;   // Breite (Halbwert)
  float errBigMin      = 25.0f;   // ab 25° = "groß"

  // ── Speed-Grenzen (PWM-artige Einheiten) ──
  float speedStoppedMax = 30.0f;  // unter 20 = "steht"
  float speedSlowCenter = 40.0f;  // Zentrum "langsam"
  float speedSlowWidth  = 35.0f;  // Breite
  float speedFastMin    = 80.0f;  // ab 60 = "schnell"

  // ── Ausgabe-Matrix [speed][error] ──
  //    Zeilen: STOPPED, SLOW, FAST
  //    Spalten: SMALL, MEDIUM, BIG
  float rules[3][3] = {
    { 0.40f, 0.80f, 1.00f },   // STOPPED
    { 0.30f, 0.60f, 0.90f },   // SLOW
    { 0.15f, 0.40f, 0.80f },   // FAST
  };

  // ── Minimale Autorität (nie komplett 0) ──
  float minAuthority = 0.10f;
};

// ═══════════════════════════════════════════════════════════════════
class RotationAuthority {
public:

  RotationAuthority() : cfg_() {}
  explicit RotationAuthority(const RotAuthConfig& c) : cfg_(c) {}

  // ──────── Hauptfunktion: skaliert rotCmd ────────────────────
  //  rotCmd       = Ausgang vom Gyro-PID
  //  headingErr   = aktueller Heading-Fehler in Grad (vorzeichenbehaftet)
  //  transSpeed   = Betrag des Translationsvektors (√(vx²+vy²))
  //
  //  Rückgabe: rotCmd × authority   (authority ∈ [minAuth, 1.0])
  float apply(float rotCmd, float headingErr, float transSpeed) const {
    float authority = computeAuthority(fabsf(headingErr), transSpeed);
    return rotCmd * authority;
  }

  // Nur Authority berechnen (für Debug / Teleplot)
  float computeAuthority(float absErr, float speed) const {
    // ── 1. Fuzzification: Heading-Error ──────────────────────
    float muErrSmall  = falling(absErr, 0.0f, cfg_.errSmallMax);
    float muErrMedium = bell(absErr, cfg_.errMediumCenter, cfg_.errMediumWidth);
    float muErrBig    = rising(absErr, cfg_.errBigMin - 8.0f, cfg_.errBigMin);

    // Normalisieren (damit Summe ≈ 1)
    float errSum = muErrSmall + muErrMedium + muErrBig;
    if (errSum < 0.001f) errSum = 1.0f;
    muErrSmall  /= errSum;
    muErrMedium /= errSum;
    muErrBig    /= errSum;

    // ── 2. Fuzzification: Translation-Speed ──────────────────
    float muStopped = falling(speed, 0.0f, cfg_.speedStoppedMax);
    float muSlow    = bell(speed, cfg_.speedSlowCenter, cfg_.speedSlowWidth);
    float muFast    = rising(speed, cfg_.speedFastMin - 15.0f, cfg_.speedFastMin);

    float spdSum = muStopped + muSlow + muFast;
    if (spdSum < 0.001f) spdSum = 1.0f;
    muStopped /= spdSum;
    muSlow    /= spdSum;
    muFast    /= spdSum;

    // ── 3. Regelauswertung (gewichteter Mittelwert) ──────────
    //    Jede Kombination (speed_i, err_j) hat Gewicht = min(mu_speed_i, mu_err_j)
    //    und liefert den Ausgabewert aus der Matrix.
    float muSpeed[3] = { muStopped, muSlow, muFast };
    float muErr[3]   = { muErrSmall, muErrMedium, muErrBig };

    float weightedSum = 0.0f;
    float weightTotal = 0.0f;

    for (int s = 0; s < 3; s++) {
      for (int e = 0; e < 3; e++) {
        float w = fminf(muSpeed[s], muErr[e]);   // AND = min
        weightedSum += w * cfg_.rules[s][e];
        weightTotal += w;
      }
    }

    float authority = (weightTotal > 0.001f)
                    ? (weightedSum / weightTotal)
                    : 1.0f;

    // Clamp
    if (authority < cfg_.minAuthority) authority = cfg_.minAuthority;
    if (authority > 1.0f) authority = 1.0f;

    return authority;
  }

  // Zugriff auf Config (für Runtime-Tuning)
  RotAuthConfig& config() { return cfg_; }
  const RotAuthConfig& config() const { return cfg_; }

private:
  RotAuthConfig cfg_;

  // ─────────── Membership-Funktionen ───────────────────────────
  //
  //  falling(x, lo, hi):  1.0 bei x≤lo,  0.0 bei x≥hi,  linear dazwischen
  //     1 ┤████
  //       │    ████
  //     0 ┤        ████────
  //       └──lo──hi──────→ x
  //
  static float falling(float x, float lo, float hi) {
    if (x <= lo) return 1.0f;
    if (x >= hi) return 0.0f;
    return (hi - x) / (hi - lo);
  }

  //  rising(x, lo, hi):  0.0 bei x≤lo,  1.0 bei x≥hi,  linear dazwischen
  //     1 ┤        ████████
  //       │    ████
  //     0 ┤────────
  //       └──lo──hi──────→ x
  //
  static float rising(float x, float lo, float hi) {
    if (x <= lo) return 0.0f;
    if (x >= hi) return 1.0f;
    return (x - lo) / (hi - lo);
  }

  //  bell(x, center, width):  Dreieck mit Spitze bei center, Basis ±width
  //     1 ┤    ▲
  //       │   ███
  //     0 ┤──/   \──
  //       └──c-w──c──c+w──→ x
  //
  static float bell(float x, float center, float width) {
    float d = fabsf(x - center);
    if (d >= width) return 0.0f;
    return 1.0f - (d / width);
  }
};