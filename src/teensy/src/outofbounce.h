#pragma once
#include <math.h>

#ifndef SAFETY_VEC2_DEFINED
#define SAFETY_VEC2_DEFINED 1
struct Vec2 {
  float x;
  float y;
};
#endif

/**
 * Out-of-Bounds / Field-Bounds Safety (RCJ Soccer 2vs2 Open)
 *
 * WICHTIG: Alle Distanzen/Positionen hier sind in **cm** gedacht.
 *          Wenn deine Pose aus dem LiDAR in mm kommt (Player.x/y),
 *          vor dem Aufruf umrechnen:  px_cm = Player.x * 0.1f;
 *
 * cmd.x/cmd.y sind Speed-Kommandos in deinem DriveSystem-Einheitenraum.
 * Die Logik begrenzt NUR die Komponente, die Richtung "aus dem Feld" fährt,
 * und erzeugt in der Hard-Zone einen definierten Rückstoß nach innen.
 */
struct BoundsConfig {
  float xLimit;      // Feld-Halbmaß X in cm (z.B. 90)
  float yLimit;      // Feld-Halbmaß Y in cm (z.B. 120)

  float softMargin;  // cm: ab (Limit-softMargin) beginnt Soft-Limit
  float hardMargin;  // cm: ab (Limit-hardMargin) beginnt Push (muss < softMargin sein)

  float kPush;       // Push-Gain: push ≈ kPush * pen^2, saturiert auf maxHard
  float maxSoft;     // max OUTWARD-Speed am Anfang Soft-Zone
  float maxHard;     // max Push / min Return-Speed bei Outside
};

struct BoundsExtras {
  float yEscapeThresh = 20.0f;   // cm (lokale Bot-Koords)
  float xEscapeThresh = 30.0f;   // cm (lokale Bot-Koords)
  bool  enableDirectionalEscape = true;

  // Escape lockert NUR Soft-Zone (nie Hard-Zone)
  float escapeSoftFactor = 0.6f; // 1.0 = keine Änderung, 0.6 = Soft-Start näher an Linie
  float minSoftHardGap   = 2.0f; // cm Mindestabstand Soft->Hard selbst im Escape

  float outsidePanicCm   = 2.0f; // cm außerhalb => Panic Return erzwingen
};

void applyFieldBounds(Vec2& cmd,
                      float px, float py,          // cm!
                      const BoundsConfig& cfg,
                      const Vec2& ballLocal,       // cm! (für Escape-Heuristik)
                      const BoundsExtras& extras = BoundsExtras{});
