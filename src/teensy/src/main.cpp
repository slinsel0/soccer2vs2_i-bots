#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "DriveSystem.h"
#include "GyroSystem.h"
#include "LidarProcessing.h"
#include "outofbounce.h"

#include <PacketSerial.h>
#include <FastCRC.h>
#include <PIDController.h>

#include "state_manager.h"

// -------------------- Comms --------------------
COBSPacketSerial cobsSerial;
FastCRC32 CRC32;

// Packet aus deinem Code (Ball/Command)
struct __attribute__((packed)) VectorCmd {
  uint8_t  msg_id;
  uint8_t  seq;
  uint32_t t_us;
  float    vx;
  float    vy;
  float    omega;
  uint32_t crc32;
};

static volatile float    last_vx = 0, last_vy = 0, last_omega = 0;
static volatile bool     got_cmd = false;
static volatile uint32_t last_cmd_ms = 0;
static constexpr uint32_t CMD_TIMEOUT_MS = 200;

void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < sizeof(VectorCmd)) return;

  uint32_t calc = CRC32.crc32(buffer, sizeof(VectorCmd) - 4);

  VectorCmd pkt;
  memcpy(&pkt, buffer, sizeof(VectorCmd));
  if (calc != pkt.crc32) return;
  if (pkt.msg_id != 1) return;

  last_vx = pkt.vx;
  last_vy = pkt.vy;
  last_omega = pkt.omega;
  got_cmd = true;
  last_cmd_ms = millis();

  // Ack
  uint8_t ack_body[2] = { 0xA1, pkt.seq };
  uint32_t ack_crc = CRC32.crc32(ack_body, sizeof(ack_body));
  uint8_t ack_frame[6];
  memcpy(&ack_frame[0], ack_body, 2);
  memcpy(&ack_frame[2], &ack_crc, 4);
  cobsSerial.send(ack_frame, sizeof(ack_frame));
}

// -------------------- Subsystems --------------------
GyroSystem  gyro;
DriveSystem Drive;

// -------------------- PID / Tuning --------------------
// Heading PID: timeStepMs MUSS zur IMU-Rate passen (BNO08x 100Hz -> 10ms)
PIDController pidHeading(5.5f, 0.001f, 0.08f, 10, 25.0f);

// Center Hold (Pose in mm!)
// Center Hold (Pose in mm!)
// Kp = 0.5f -> 100mm error => 50 output (gut bewegbar in DriveSystem Skalierung)
PIDController pidCenterX(0.5f, 0.0f, 0.0f, 20, 200.0f);
PIDController pidCenterY(0.5f, 0.0f, 0.0f, 20, 200.0f);

// Ball Control (Ball in Pixel / local units)
PIDController pidBallX(0.6f, 0.01f, 0.2f, 20, 100.0f);
PIDController pidBallY(0.6f, 0.01f, 0.2f, 20, 100.0f);

// -------------------- Bounds --------------------
static const BoundsConfig kBounds = {
  /* xLimit      */ 80.0f,  // Erhöht von 70.0 (mehr Spielfeld)
  /* yLimit      */ 110.0f, // Erhöht von 90.0 (mehr Spielfeld)
  /* softMargin  */ 20.0f,
  /* hardMargin  */ 10.0f,
  /* kPush       */ 1.5f,
  /* maxSoft     */ 200.0f,
  /* maxHard     */ 80.0f
};

// Escape im bisherigen Setup NICHT aktivieren, solange ballLocal nicht in cm ist.
static const BoundsExtras kExtras = {
  /* yEscapeThresh           */ 200.0f,
  /* xEscapeThresh           */ 200.0f,
  /* enableDirectionalEscape */ false 
};

// -------------------- State --------------------
StateManager sm;

// -------------------- Task Scheduler --------------------
struct PeriodicTask {
  uint32_t period_us;
  uint32_t next_us;

  void init(uint32_t now_us) { next_us = now_us + period_us; }

  bool ready(uint32_t now_us) {
    // signed wrap-safe compare
    if ((int32_t)(now_us - next_us) >= 0) {
      next_us += period_us;
      return true;
    }
    return false;
  }
};

static PeriodicTask tIMU      { 10000, 0 };  // 100 Hz
static PeriodicTask tBEHAVIOR { 20000, 0 };  // 50 Hz
static PeriodicTask tCONTROL  {  5000, 0 };  // 200 Hz
static PeriodicTask tTEL      { 40000, 0 };  // 25 Hz

// -------------------- Helper --------------------
static Vec2 computeBehindBallTargetPx(float ballX, float ballY) {
  // dein bisheriges Konzept: hinter den Ball
  Vec2 offset = {0.0f, 0.0f};

  if (ballY > 0.0f) {
    if (fabsf(ballX) < 100.0f) {
      offset.x = 0.0f; offset.y = 0.0f;
    } else {
      offset.y = 800.0f;
    }
  } else {
    offset.x = (ballX < 0.0f) ? -500.0f : 500.0f;
    offset.y = 750.0f;
  }
  return { ballX - offset.x, ballY - offset.y };
}

static void resetControllersOnStateChange(RobotState from, RobotState to) {
  // Beim Wechsel: Integrale/History sauber resetten (stabiler)
  (void)from;

  pidHeading.reset();
  pidCenterX.reset();
  pidCenterY.reset();
  pidBallX.reset();
  pidBallY.reset();
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(2000000);

  cobsSerial.setStream(&Serial);
  cobsSerial.setPacketHandler(&onPacketReceived);

  // LiDAR UART Buffer (hilft gegen Overflow)
  // Falls du große Drops siehst, hier erhöhen.
  Serial1.addMemoryForRead((void*)malloc(2048), 2048);

  gyro.begin();
  LidarBegin();

  sm.begin(millis());

  uint32_t now_us = micros();
  tIMU.init(now_us);
  tBEHAVIOR.init(now_us);
  tCONTROL.init(now_us);
  tTEL.init(now_us);
}

// -------------------- Main Loop --------------------
void loop() {
  // 1) Always service comms + lidar read
  cobsSerial.update();   // non-blocking
  lidaar();              // enthält lidarsensor.loop + process (bei dir aktuell jedes Loop)

  const uint32_t now_ms = millis();
  const uint32_t now_us = micros();

  // 2) Build context
  RobotContext ctx;
  ctx.now_ms = now_ms;

  // IMU update auf Rate (stabil)
  if (tIMU.ready(now_us)) {
    gyro.update();
  }
  ctx.heading_deg = gyro.getAngleDegrees();

  // Pose aus LiDAR
  ctx.px_mm = Player.x;
  ctx.py_mm = Player.y;
  ctx.px_cm = ctx.px_mm * 0.1f;   // mm -> cm (WICHTIG für Bounds)
  ctx.py_cm = ctx.py_mm * 0.1f;

  // Ball validity aus comm timeout
  const bool commFresh = got_cmd && ((now_ms - last_cmd_ms) <= CMD_TIMEOUT_MS);
  ctx.ball_valid = commFresh;
  ctx.ball_raw_x = last_vx;
  ctx.ball_raw_y = last_vy;
  

  // Health (hier erstmal "true"; später sauber über last-update timestamps)
  ctx.imu_ok = true;
  ctx.lidar_ok = true;
  ctx.comm_ok = true;

  // 3) State update (Behavior rate)
  static RobotState prevState = RobotState::BOOT;
  if (tBEHAVIOR.ready(now_us)) {
    RobotState st = sm.update(ctx, kBounds);
    if (st != prevState) {
      resetControllersOnStateChange(prevState, st);
      prevState = st;
    }
  }

  // 4) Control + Actuation (high-rate)
  if (tCONTROL.ready(now_us)) {
    const RobotState st = sm.state();
    BehaviorCommand bc = sm.behavior(ctx);

    // --- Translation Command ---
    Vec2 v_cmd = {0.0f, 0.0f};

    if (st == RobotState::CHASE_BALL && ctx.ball_valid) {
      // Ball lokal in Pixel: center offset (deine bisherigen Werte)
      const float bx = ctx.ball_raw_x - 700.0f;
      const float by = ctx.ball_raw_y - 400.0f;

      // Serial.print("Ball: ");
      // Serial.print(bx);
      // Serial.print(" ");
      // Serial.println(by);

      //Vec2 target = computeBehindBallTargetPx(bx, by);

      v_cmd.x = pidBallX.update(bx);
      v_cmd.y = pidBallY.update(by);
    }
    else if (st == RobotState::RECOVER_BOUNDS || st == RobotState::RETURN_CENTER) {
      // sanft Richtung Mitte (Pose in mm)
      // PID rechnet in Feldkoordinaten (Fehler = Mitte - Ist = -ctx.px_mm)
      float v_field_x = pidCenterX.update(-ctx.px_mm);
      float v_field_y = pidCenterY.update(-ctx.py_mm);

      // In Roboter-Koordinaten rotieren (da DriveSystem v_robot erwartet)
      // theta = -heading (weil wir Feld -> Bot wollen, und Heading Bot->Feld ist)
      float theta = -ctx.heading_deg * (M_PI / 180.0f);
      float c = cosf(theta);
      float s = sinf(theta);

      v_cmd.x = v_field_x * c - v_field_y * s;
      v_cmd.y = v_field_x * s + v_field_y * c;
    }
    else {
      v_cmd = bc.v; // IDLE/SEARCH/FAILSAFE default
    }

    // --- Apply field bounds (units in cm!) ---
    // ballLocal ist hier NICHT cm -> Escape deaktiviert. Übergib 0.
    Vec2 dummyBallLocalCm = {0.0f, 0.0f};
    //applyFieldBounds(v_cmd, ctx.px_cm, ctx.py_cm, kBounds, dummyBallLocalCm, kExtras);

    // --- Omega / Heading ---
    float omega_cmd = 0.0f;

    omega_cmd = pidHeading.update(ctx.heading_deg);
  

    // 5) Drive
    Drive.calcDrive(-v_cmd.x, v_cmd.y, -omega_cmd);
Drive .drive();  
  }

  // 6) Telemetry (clean, rate-limited)
  if (tTEL.ready(now_us)) {
    Serial.print(">state:");
    Serial.print(sm.stateName());
    Serial.print(",px_mm:");
    Serial.print(Player.x, 1);
    Serial.print(",py_mm:");
    Serial.print(Player.y, 1);
    Serial.print(",heading_deg:");
    Serial.print(gyro.getAngleDegrees(), 2);
    Serial.print(",ball_ok:");
    Serial.print((int)got_cmd);
    Serial.print(",ball_x:");
    Serial.print((float)last_vx, 1);
    Serial.print(",ball_y:");
    Serial.println((float)last_vy, 1);
  }
}
