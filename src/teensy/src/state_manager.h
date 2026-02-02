#pragma once
#include <Arduino.h>
#include <math.h>
#include "outofbounce.h"   // Vec2, BoundsConfig

// ---------- Utilities ----------
static inline float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

static inline bool isOutsideSoft(float px_cm, float py_cm, const BoundsConfig& b) {
  return (fabsf(px_cm) > b.xLimit) || (fabsf(py_cm) > b.yLimit);
}

// ---------- Context / IO ----------
struct RobotContext {
  uint32_t now_ms = 0;

  // Pose
  float heading_deg = 0.0f; // -180..180
  float px_mm = 0.0f;
  float py_mm = 0.0f;
  float px_cm = 0.0f;
  float py_cm = 0.0f;

  // Vision / Ball (dein Packet: last_vx/last_vy sind in der Praxis "Ball" Werte)
  bool  ball_valid = false;
  float ball_raw_x = 0.0f;  // z.B. Pixel
  float ball_raw_y = 0.0f;  // z.B. Pixel

  // Health
  bool imu_ok = true;
  bool lidar_ok = true;
  bool comm_ok = true;
};

struct BehaviorCommand {
  // gewünschte Translation (DriveSystem Units)
  Vec2 v = {0.0f, 0.0f};

  // Heading-Setpoint
  bool  hold_heading = true;
  float heading_set_deg = 0.0f;

  // Optional: Open-loop omega für SEARCH
  bool  use_openloop_omega = false;
  float omega_openloop = 0.0f;
};

// ---------- State Machine ----------
enum class RobotState : uint8_t {
  BOOT = 0,
  IDLE,
  SEARCH_BALL,
  CHASE_BALL,
  RETURN_CENTER,
  RECOVER_BOUNDS,
  FAILSAFE_STOP
};

class StateManager {
public:
  void begin(uint32_t now_ms) {
    state_ = RobotState::BOOT;
    state_enter_ms_ = now_ms;
  }

  RobotState state() const { return state_; }

  const char* stateName() const {
    switch (state_) {
      case RobotState::BOOT:          return "BOOT";
      case RobotState::IDLE:          return "IDLE";
      case RobotState::SEARCH_BALL:   return "SEARCH_BALL";
      case RobotState::CHASE_BALL:    return "CHASE_BALL";
      case RobotState::RETURN_CENTER: return "RETURN_CENTER";
      case RobotState::RECOVER_BOUNDS:return "RECOVER_BOUNDS";
      case RobotState::FAILSAFE_STOP: return "FAILSAFE_STOP";
      default: return "UNKNOWN";
    }
  }

  // Tick: entscheidet nur Transitionen, keine Regler
  RobotState update(const RobotContext& ctx,
                    const BoundsConfig& bounds)
  {
    const bool outside = isOutsideSoft(ctx.px_cm, ctx.py_cm, bounds);

    // Hard failsafe
    if (!ctx.imu_ok || !ctx.lidar_ok) {
      transition(RobotState::FAILSAFE_STOP, ctx.now_ms);
      return state_;
    }

    // Out of bounds => recovery hat Priorität
    if (outside) {
      transition(RobotState::RECOVER_BOUNDS, ctx.now_ms);
      return state_;
    }

    switch (state_) {
      case RobotState::BOOT:
        if (ctx.now_ms - state_enter_ms_ > 300) {
          transition(RobotState::IDLE, ctx.now_ms);
        }
        break;

      case RobotState::IDLE:
        if (ctx.ball_valid) transition(RobotState::CHASE_BALL, ctx.now_ms);
        else if (ctx.now_ms - state_enter_ms_ > 500) transition(RobotState::SEARCH_BALL, ctx.now_ms);
        break;

      case RobotState::SEARCH_BALL:
        if (ctx.ball_valid) transition(RobotState::CHASE_BALL, ctx.now_ms);
        break;

      case RobotState::CHASE_BALL:
        if (!ctx.ball_valid) transition(RobotState::SEARCH_BALL, ctx.now_ms);
        break;

      case RobotState::RETURN_CENTER:
        // optional: wenn du das später nutzen willst
        if (ctx.ball_valid) transition(RobotState::CHASE_BALL, ctx.now_ms);
        break;

      case RobotState::RECOVER_BOUNDS:
        // Recovery endet automatisch, sobald wieder im Feld
        // (outside wird oben abgefangen)
        transition(ctx.ball_valid ? RobotState::CHASE_BALL : RobotState::SEARCH_BALL, ctx.now_ms);
        break;

      case RobotState::FAILSAFE_STOP:
        // Wenn wieder healthy, zurück
        if (ctx.imu_ok && ctx.lidar_ok) transition(RobotState::IDLE, ctx.now_ms);
        break;
    }
    return state_;
  }

  // Behavior: produziert Ziele (keine PID hier drin)
  BehaviorCommand behavior(const RobotContext& ctx) const {
    BehaviorCommand cmd;

    switch (state_) {
      case RobotState::BOOT:
      case RobotState::IDLE:
        cmd.v = {0, 0};
        cmd.hold_heading = true;
        cmd.heading_set_deg = 0.0f;
        break;

      case RobotState::SEARCH_BALL:
        cmd.v = {0, 0};
        cmd.hold_heading = false;
        cmd.use_openloop_omega = true;
        cmd.omega_openloop = 0.8f;     // Suchrotation (DriveSystem-Einheit)
        break;

      case RobotState::CHASE_BALL:
        // Translation wird im main.cpp über "Ball Controller" gerechnet
        cmd.v = {0, 0};                // placeholder
        cmd.hold_heading = true;
        cmd.heading_set_deg = 0.0f;    // im Moment: "Heading Hold auf 0°"
        break;

      case RobotState::RETURN_CENTER:
      case RobotState::RECOVER_BOUNDS:
        // Translation wird im main.cpp über "Center Controller" gerechnet
        cmd.v = {0, 0};
        cmd.hold_heading = true;
        cmd.heading_set_deg = 0.0f;
        break;

      case RobotState::FAILSAFE_STOP:
        cmd.v = {0, 0};
        cmd.hold_heading = false;
        cmd.use_openloop_omega = false;
        cmd.omega_openloop = 0.0f;
        break;
    }

    return cmd;
  }

private:
  void transition(RobotState next, uint32_t now_ms) {
    if (next == state_) return;
    state_ = next;
    state_enter_ms_ = now_ms;
  }

  RobotState state_ = RobotState::BOOT;
  uint32_t state_enter_ms_ = 0;
};
