#include "outofbounce.h"
#include <math.h>

// Smoothstep [0..1] — sanftes Ein- und Ausblenden, S-Kurve t²(3-2t).
// Bei kleinen t deutlich stärker dämpfend als linear, bei großen t milder.
static inline float smoothstep01(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

// Predictive damping: effektiver Margin wächst mit Geschwindigkeit
// in Richtung Wand → der Bot beginnt früher zu bremsen, wenn er
// schnell auf die Wand zufährt.
static inline float effectiveMargin(float baseMargin, float vTowardWall, float lookAhead) {
    if (vTowardWall <= 0.0f) return baseMargin;
    return baseMargin + lookAhead * vTowardWall;
}

void applyFieldBounds(Vec2& cmd, float px, float py, const BoundsConfig& cfg) {

    // ═══════════════════════════════════════════════════════════
    //  0. PENALTY AREA (eigene Hälfte, -Y): aktiv heraustreiben
    //     PA = { |x| < penaltyHalfWidth, py < penaltyLineY }
    //     → Y wird auf eine positive Mindest-Auswurf-Geschwindigkeit
    //       gezwungen, deren Stärke mit der Eindringtiefe wächst.
    //     → X-Befehl bleibt unangetastet, damit der Bot seitlich
    //       ausweichen / dem Ball folgen kann, während er rausfährt.
    // ═══════════════════════════════════════════════════════════
    {
        float penaltyLineY = -cfg.yLimit + cfg.penaltyDepth;
        bool inOwnPenalty = (fabsf(px) < cfg.penaltyHalfWidth) && (py < penaltyLineY);

        if (inOwnPenalty) {
            float depth = penaltyLineY - py;          // ≥0, je tiefer drin desto größer
            float exitY = cfg.penaltyExitSpeed + depth * cfg.penaltyExitKp;
            if (cmd.y < exitY) cmd.y = exitY;
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  1. X-ACHSE (Seitenwände)
    //     - Außerhalb safeLine  → P-Pullback
    //     - Innerhalb           → Smoothstep-Damping mit Predictive Margin
    // ═══════════════════════════════════════════════════════════
    float safeLineX = cfg.xLimit - cfg.safeMarginX;

    if (px > safeLineX) {
        float pull = (safeLineX - px) * cfg.returnKp;
        if (cmd.x > pull) cmd.x = pull;
    }
    else if (px < -safeLineX) {
        float pull = (-safeLineX - px) * cfg.returnKp;
        if (cmd.x < pull) cmd.x = pull;
    }
    else {
        // rechte Wand
        if (cmd.x > 0.0f) {
            float distRight = safeLineX - px;
            float marginR   = effectiveMargin(cfg.dampingMargin, cmd.x, cfg.speedLookAhead);
            cmd.x *= smoothstep01(distRight / marginR);
        }
        // linke Wand
        if (cmd.x < 0.0f) {
            float distLeft = px - (-safeLineX);
            float marginL  = effectiveMargin(cfg.dampingMargin, -cmd.x, cfg.speedLookAhead);
            cmd.x *= smoothstep01(distLeft / marginL);
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  2. Y-ACHSE (Tor oder Wand)
    // ═══════════════════════════════════════════════════════════
    float safeLineY;
    if (fabsf(px) <= cfg.goalHalfWidth) {
        safeLineY = cfg.yLimit - cfg.goalSafeMarginY; // Torzone
    } else {
        safeLineY = cfg.yLimit - cfg.safeMarginY;     // Normale Wand
    }

    if (py > safeLineY) {
        float pull = (safeLineY - py) * cfg.returnKp;
        if (cmd.y > pull) cmd.y = pull;
    }
    else if (py < -safeLineY) {
        float pull = (-safeLineY - py) * cfg.returnKp;
        if (cmd.y < pull) cmd.y = pull;
    }
    else {
        // obere Wand / gegnerische Torzone
        if (cmd.y > 0.0f) {
            float distTop = safeLineY - py;
            float marginT = effectiveMargin(cfg.dampingMargin, cmd.y, cfg.speedLookAhead);
            cmd.y *= smoothstep01(distTop / marginT);
        }
        // untere Wand / eigene Torzone
        if (cmd.y < 0.0f) {
            float distBot = py - (-safeLineY);
            float marginB = effectiveMargin(cfg.dampingMargin, -cmd.y, cfg.speedLookAhead);
            cmd.y *= smoothstep01(distBot / marginB);
        }
    }

    // ═══════════════════════════════════════════════════════════
    //  3. ABSOLUTE GESCHWINDIGKEITSBEGRENZUNG
    // ═══════════════════════════════════════════════════════════
    constexpr float ABSOLUTE_MAX = 250.0f;
    float vSq = cmd.x * cmd.x + cmd.y * cmd.y;
    if (vSq > ABSOLUTE_MAX * ABSOLUTE_MAX) {
        float scale = ABSOLUTE_MAX / sqrtf(vSq);
        cmd.x *= scale;
        cmd.y *= scale;
    }
}
