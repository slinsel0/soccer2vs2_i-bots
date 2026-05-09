#include "ballTrajectory.h"

// ─────────────────────────────────────────────────────────────
// Frame-Konventionen
//
//   Kamera-Frame (bx, by):  bx = last_vx - CAM_CENTER_X
//                           by = last_vy - CAM_CENTER_Y
//     +bx = bot rechts, +by = unten im Bild = bot hinten
//     d.h. -by = "vorne" (siehe computeBehindBallTarget im Original)
//
//   Natural-Frame  (bxN, byN): bxN = bx, byN = -by
//     +bxN = bot rechts, +byN = bot vorne
//
//   Stable-Frame   (bxS, byS): natural rotiert mit -heading
//     → +YS zeigt immer auf das gegnerische Tor (Welt-Orientierung)
//
//   Heading-Konvention: g_g = 0 → bot blickt entlang +Y_world
//                       g_g > 0 → im Uhrzeigersinn von oben gesehen
// ─────────────────────────────────────────────────────────────

static inline void camToStable(float bxCam, float byCam, float th,
                               float& bxs, float& bys) {
    float bxN =  bxCam;
    float byN = -byCam;
    float c = cosf(th), s = sinf(th);
    // Welt = R_cw(th) * Natural
    bxs =  bxN * c + byN * s;
    bys = -bxN * s + byN * c;
}

static inline void stableToCam(float bxs, float bys, float th,
                               float& bxCam, float& byCam) {
    float c = cosf(th), s = sinf(th);
    // Natural = R_ccw(th) * Welt
    float bxN = bxs * c - bys * s;
    float byN = bxs * s + bys * c;
    bxCam =  bxN;
    byCam = -byN;
}

void btReset(BallTrajectoryState& st) {
    st.bxStable = 0.0f;
    st.byStable = 0.0f;
    st.vxStable = 0.0f;
    st.vyStable = 0.0f;
    st.lastMs   = 0;
    st.lastSrcMs = 0;
    st.initialized = false;
}

void btUpdate(BallTrajectoryState& st,
              const BallTrajectoryConfig& cfg,
              float bxCam, float byCam,
              float headingRad,
              uint32_t nowMs) {
    float bxs, bys;
    camToStable(bxCam, byCam, headingRad, bxs, bys);

    // Erstinitialisierung
    if (!st.initialized) {
        st.bxStable    = bxs;
        st.byStable    = bys;
        st.vxStable    = 0.0f;
        st.vyStable    = 0.0f;
        st.lastMs      = nowMs;
        st.initialized = true;
        return;
    }

    uint32_t dtMs = nowMs - st.lastMs;

    // Zu lange keine Daten → Velocity verwerfen, Position neu setzen
    if (dtMs > (uint32_t)cfg.resetGapMs) {
        st.bxStable = bxs;
        st.byStable = bys;
        st.vxStable = 0.0f;
        st.vyStable = 0.0f;
        st.lastMs   = nowMs;
        return;
    }

    // Zu schnell → Velocity nicht updaten, aber Position weiterführen
    if ((float)dtMs < cfg.minDtMs) {
        return;
    }

    float dt = (float)dtMs * 0.001f;
    float rawVx = (bxs - st.bxStable) / dt;
    float rawVy = (bys - st.byStable) / dt;

    // Ausreißer-Sättigung
    float speed = sqrtf(rawVx * rawVx + rawVy * rawVy);
    if (speed > cfg.maxBallSpeed && speed > 1e-3f) {
        float k = cfg.maxBallSpeed / speed;
        rawVx *= k;
        rawVy *= k;
    }

    // EMA-Tiefpass auf die Velocity
    float a = cfg.emaAlpha;
    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    st.vxStable = st.vxStable * (1.0f - a) + rawVx * a;
    st.vyStable = st.vyStable * (1.0f - a) + rawVy * a;

    st.bxStable = bxs;
    st.byStable = bys;
    st.lastMs   = nowMs;
}

Vec2 btApproachTargetCam(const BallTrajectoryState& st,
                         const BallTrajectoryConfig& cfg,
                         float headingRad) {
    Vec2 out = { 0.0f, 0.0f };

    if (!st.initialized) {
        // Sicherer Fallback: in den letzten bekannten Pixel zielen,
        // ohne Lookahead/Lead.
        return out;
    }

    // 1) Predicted Ball Position im stabilen Frame
    float la = cfg.lookAheadMs * 0.001f;
    float bxFut = st.bxStable + st.vxStable * la;
    float byFut = st.byStable + st.vyStable * la;

    float tgtxS, tgtyS;

    if (byFut < cfg.alignFrontMin) {
        // ─── Ball ist NICHT nördlich vom Bot ──────────────────────
        // Direkter Süd-Anflug würde durch den Ball laufen.
        // Stattdessen LATERAL umkreisen: Ziel entgegen der Ballseite,
        // mit moderatem Süd-Versatz, damit der Bot diagonal an der
        // Ballseite vorbeischwingt. Mecanum erlaubt das in einem Zug.
        float side = (bxFut >= 0.0f) ? -1.0f : 1.0f;
        tgtxS = bxFut + side * cfg.approachDistance;
        tgtyS = byFut - cfg.approachDistance * 0.5f;
    } else {
        // ─── Ball nördlich vom Bot: weiches Blending ──────────────
        // align=1 → in der Schusslinie → durch den Ball durchstoßen
        // align=0 → muss noch zentriert / hinten dran → voller Versatz
        float ax = 1.0f - fminf(fabsf(bxFut) / cfg.alignWidth, 1.0f);
        float align = ax * ax * (3.0f - 2.0f * ax);   // smoothstep

        float effApproach = cfg.approachDistance * (1.0f - align);
        float lateralLead = st.vxStable * la * cfg.lateralLeadGain * (1.0f - align);

        tgtxS = bxFut + lateralLead;
        tgtyS = byFut - effApproach;
    }

    // 2) Zurück in den Kamera-Frame für die bestehende PID-Pipeline
    stableToCam(tgtxS, tgtyS, headingRad, out.x, out.y);
    return out;
}
