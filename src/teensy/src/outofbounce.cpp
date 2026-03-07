#include "outofbounce.h"
#include <math.h>

void applyFieldBounds(Vec2& cmd, float px, float py, const BoundsConfig& cfg) {
    
    // ═══════════════════════════════════════════════════════════
    //  1. X-ACHSE (Seitenwände)
    // ═══════════════════════════════════════════════════════════
    float safeLineX = cfg.xLimit - cfg.safeMarginX;
    
    if (px > safeLineX) {
        // Rechts ÜBER der Grenze -> P-Regler zieht zurück nach links (negativ)
        float pull = (safeLineX - px) * cfg.returnKp; 
        if (cmd.x > pull) cmd.x = pull; // Erlaube nur Flucht nach links
    } 
    else if (px < -safeLineX) {
        // Links ÜBER der Grenze -> P-Regler zieht zurück nach rechts (positiv)
        float pull = (-safeLineX - px) * cfg.returnKp; 
        if (cmd.x < pull) cmd.x = pull; // Erlaube nur Flucht nach rechts
    }
    else {
        // DAMPING: Kurz vor der Linie weich abbremsen
        float distToRight = safeLineX - px;
        if (distToRight < cfg.dampingMargin && cmd.x > 0) cmd.x *= (distToRight / cfg.dampingMargin);
        
        float distToLeft = px - (-safeLineX);
        if (distToLeft < cfg.dampingMargin && cmd.x < 0) cmd.x *= (distToLeft / cfg.dampingMargin);
    }

    // ═══════════════════════════════════════════════════════════
    //  2. Y-ACHSE (Dynamisch: Tor oder Wand)
    // ═══════════════════════════════════════════════════════════
    float safeLineY;
    if (fabsf(px) <= cfg.goalHalfWidth) {
        safeLineY = cfg.yLimit - cfg.goalSafeMarginY; // Torzone
    } else {
        safeLineY = cfg.yLimit - cfg.safeMarginY;     // Normale Wand
    }

    if (py > safeLineY) {
        // Oben ÜBER der Grenze -> P-Regler zieht zurück nach unten (negativ)
        float pull = (safeLineY - py) * cfg.returnKp; 
        if (cmd.y > pull) cmd.y = pull;
    } 
    else if (py < -safeLineY) {
        // Unten ÜBER der Grenze -> P-Regler zieht zurück nach oben (positiv)
        float pull = (-safeLineY - py) * cfg.returnKp; 
        if (cmd.y < pull) cmd.y = pull;
    }
    else {
        // DAMPING in Y-Richtung
        float distToTop = safeLineY - py;
        if (distToTop < cfg.dampingMargin && cmd.y > 0) cmd.y *= (distToTop / cfg.dampingMargin);
        
        float distToBottom = py - (-safeLineY);
        if (distToBottom < cfg.dampingMargin && cmd.y < 0) cmd.y *= (distToBottom / cfg.dampingMargin);
    }

    // ═══════════════════════════════════════════════════════════
    //  3. ABSOLUTE GESCHWINDIGKEITSBEGRENZUNG (Safety)
    // ═══════════════════════════════════════════════════════════
    constexpr float ABSOLUTE_MAX = 250.0f;
    float vSq = cmd.x * cmd.x + cmd.y * cmd.y;
    if (vSq > ABSOLUTE_MAX * ABSOLUTE_MAX) {
        float scale = ABSOLUTE_MAX / sqrtf(vSq);
        cmd.x *= scale;
        cmd.y *= scale;
    }
}