
#pragma once
#include <math.h>

// Einfache 2D-Vector-Struktur
#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 {
    float x;
    float y;
};
#endif
// Tuning-Parameter für die Spielfeld-Grenzlogik
struct BoundsConfig {
  float xLimit;        // halbe kurze Seite (x)
  float yLimit;        // halbe lange Seite (y)

  // getrennte Margins für X- und Y-Richtung
  float softMargin;   // ab diesem Abstand zur X-Grenze: sanfte Drossel
  // float softMarginY;   // ab diesem Abstand zur Y-Grenze: sanfte Drossel
  // float hardMarginX;   // noch näher/über X-Grenze: harter Rückstoß
  float hardMargin;   // noch näher/über Y-Grenze: harter Rückstoß

  float kPush;         // P-Verstärkung für den Rückstoß ins Feld
  float maxSoft;       // max. Achs-Speed in Soft-Zone
  float maxHard;       // max. Achs-Speed in Hard-Zone
};


// Zusatz-Logik für "intelligentes" Entkommen (Escape)
struct BoundsExtras {
  // Y-Richtungs-Escape:
  // Wenn du VORN bist (py>0) und der Ball deutlich HINTER dir (ballLocal.y < -yEscapeThresh)
  // ODER wenn du HINTEN bist (py<0) und der Ball deutlich VOR dir (ballLocal.y > +yEscapeThresh),
  // dann wird die Y-Softbremse gelockert.
  float yEscapeThresh = 20.0f;   // in cm (lokale Bot-Koords)

  // X-Richtungs-Escape: Wenn du an rechter/ linker Linie bist, der Ball aber deutlich
  // auf der GEGENÜBERLIEGENDEN Seite liegt (ballLocal.x < -xEscapeThresh bzw. > +xEscapeThresh),
  // dann wird die X-Softbremse gelockert.
  float xEscapeThresh = 30.0f;   // in cm (lokale Bot-Koords)

  bool enableDirectionalEscape = true;
};

/**
 * Dreht/kappt die Soll-Geschwindigkeit 'cmd' so, dass der Bot das Feld nicht verlässt.
 * px, py      = aktuelle Position (z. B. Player.x / Player.y) in FELD-Koordinaten (0/0 = Mitte).
 * ballLocal   = Ballposition in LOKALEN Bot-Koordinaten (Bot = Ursprung, +y = vorwärts).
 * cfg         = Rand-/Speed-Tuning.
 * extras      = Escape-Regeln (optional, kann deaktiviert werden).
 */
void applyFieldBounds(Vec2& cmd,
                      float px, float py,
                      const BoundsConfig& cfg,
                      const Vec2& ballLocal,
                      const BoundsExtras& extras = BoundsExtras{});
