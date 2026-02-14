/*  ═══════════════════════════════════════════════════════════════════════
 *  i-bots  ·  RoboCup Junior Soccer Open  ·  Teensy 4.0 Main
 *  ═══════════════════════════════════════════════════════════════════════
 *
 *  State-Machine:
 *    NO_BALL      → Ball nicht sichtbar   → Fahre zur Feldmitte (Lidar-Pos)
 *    CHASE_BALL   → Ball sichtbar         → Fahre zum Ball (Kamera-Pixel)
 *
 *  Datenfluss Pi → Teensy (COBS-Paket):
 *    vx    = Ball-Pixel cx    (Spiegel-Kamera 1456×1088)
 *    vy    = Ball-Pixel cy
 *    omega = valid-Flag       (1.0 = gefunden, 0.0 = nicht)
 *  ═══════════════════════════════════════════════════════════════════════ */

#include <Arduino.h>
#include "DriveSystem.h"
#include "LD19.h"
#include "LidarProcessing.h"
#include <Wire.h>
#include "GyroSystem.h"
#include <PacketSerial.h>
#include <FastCRC.h>
#include <PIDController.h>
#include <math.h>
#include <outofbounce.h>

// ─────────────────────── Hardware-Objekte ───────────────────────
COBSPacketSerial cobsSerial;
FastCRC32        CRC32;
GyroSystem       gyro;
DriveSystem      Drive;

// ═══════════════════════ STATE MACHINE ══════════════════════════
enum RobotState {
  NO_BALL,        // Ball nicht sichtbar  → zurück zur Mitte
  CHASE_BALL      // Ball sichtbar        → hinfahren
};

static RobotState state = NO_BALL;

// ─────────────────── Zeitkonstanten (ms) ───────────────────────
static constexpr uint32_t BALL_LOST_TIMEOUT_MS  = 300;   // nach 300 ms ohne Ball → NO_BALL
static constexpr uint32_t CMD_TIMEOUT_MS        = 500;   // Kein Paket vom Pi     → Notfall

// ═══════════════════ PID-REGLER ════════════════════════════════
//s
//  Gyro-Heading:  Hält den Roboter auf 0° ausgerichtet
PIDController pidGyro(2.5f, 0.000f, 0.16f, /* dt_ms */ 0, /* iLim */ 25.0f);

//  Ball-Richtung (Pixel→Speed):
//    Kamera-Offset vom Spiegelzentrum (max ~±520 px) → Fahrbefehl.
//    P=0.4  → 500 px Fehler ≈ 200 Speed (Vollgas-Bereich)
//    D=0.05 → leichte Dämpfung bei schnellen Balländerungen
PIDController pidBallX(4.5f, 0.0f, 0.05f, 0, 0.0f);
PIDController pidBallY(4.5f, 0.0f, 0.05f, 0, 0.0f);

//  Return-to-Center (Lidar cm → Speed):
//    Position in cm (max ~±90).  P=2.0 → 90 cm Fehler = 180 Speed
//    D=0.3 → verhindert Überschwingen beim Ankommen
PIDController pidCenterX(4.0f, 0.0f, 0.0f, 0, 0.0f);
PIDController pidCenterY(4.0f, 0.0f, 0.0f, 0, 0.0f);

// ═══════════════════ FELDGRENZEN (Axis-Lock + Tor-Zonen) ══════
//s
//  RCJ Soccer Field 2023:  182 × 243 cm,  Tore 60 cm breit
//
//       ┌──────────┬──── 60cm ────┬──────────┐
//       │   WAND   │  BLUE GOAL   │   WAND   │  Y = +121.5
//       ├──────────┘              └──────────┤
//       │                                    │
//       │           Spielfeld                │
//       │            (0,0)                   │
//       │                                    │
//       ├──────────┐              ┌──────────┤
//       │   WAND   │ YELLOW GOAL  │   WAND   │  Y = -121.5
//       └──────────┴──── 60cm ────┴──────────┘
//     X=-91       X=-30   0   X=+30        X=+91
//
//  Y-safeLine hängt von X ab:
//    |x| > 30  →  Vor WAND     →  safeLine_Y = 121.5 - 10 = 111.5
//    |x| ≤ 30  →  Vor TOR      →  safeLine_Y = 121.5 - 30 = 91.5  (ENGER!)
//
static const BoundsConfig kBounds = {
  // ── Feldgrenzen (halbe Maße in cm) ──
  /* xLimit          */  61.0f,    // 182 / 2
  /* yLimit          */ 88.5f,    // 243 / 2

  // ── Tor-Geometrie ──
  /* goalHalfWidth   */  30.0f,    // 60 / 2

  // ── Sicherheitsmargen ──
  /* safeMarginX     */  2.0f,    // safeLine_X = 91 - 15 = 76 cm
  /* safeMarginY     */  3.0f,    // safeLine_Y (Wand) = 121.5 - 10 = 111.5 cm
  /* goalSafeMarginY */  13.0f,    // safeLine_Y (Tor)  = 121.5 - 30 = 91.5 cm
                                   //   Bot bleibt 30cm vor Torlinie → genug Platz zum Schießen

  // ── Pull-Regler ──
  /* kPull           */   15.0f,    // 10cm draussen → Speed 80
  /* maxPull         */ 180.0f     // Max Rückzug-Speed
};
// ═══════════════════ KAMERA-KONSTANTEN ═════════════════════════
//  Spiegelzentrum in Pixel (aus config.json: center_x=708, center_y=507)
static constexpr float CAM_CENTER_X = 718.0f;
static constexpr float CAM_CENTER_Y = 570.0f;

// ═══════════════════ GLOBALE VARIABLEN ═════════════════════════
float g_a = 0;           // Gyro-Heading in Grad
float p_x = 0;           // Roboter-Position X in cm (Lidar)
float p_y = 0;           // Roboter-Position Y in cm (Lidar)
const int kickerPin  = 22;  // Ausgang zur Spule
const int triggerPin = 13;  // Eingang
bool triggered = false;    // Merker, ob der Kick schon ausgelöst wurde




// ─────────────────── COBS Serial Protokoll ─────────────────────
struct __attribute__((packed)) VectorCmd {
  uint8_t  msg_id;
  uint8_t  seq;
  uint32_t t_us;
  float    vx;       // = Ball-Pixel cx
  float    vy;       // = Ball-Pixel cy
  float    omega;    // = valid-Flag (1.0 oder 0.0)   ← WICHTIG!
  uint32_t crc32;
};

volatile float    last_vx       = 0;
volatile float    last_vy       = 0;
volatile float    last_valid    = 0;     // war "last_omega" – jetzt klar benannt
volatile bool     got_cmd       = false;
volatile uint32_t last_cmd_ms   = 0;
volatile uint32_t last_ball_ms  = 0;     // Zeitstempel: letztes "Ball gesehen"

// ─────────────────── Paket-Handler (ISR-safe) ──────────────────
void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < sizeof(VectorCmd)) return;

  uint32_t calc = CRC32.crc32(buffer, sizeof(VectorCmd) - 4);
  VectorCmd pkt;
  memcpy(&pkt, buffer, sizeof(VectorCmd));

  if (calc != pkt.crc32 || pkt.msg_id != 1) return;

  last_vx    = pkt.vx;
  last_vy    = pkt.vy;
  last_valid = pkt.omega;        // 1.0 = Ball da, 0.0 = nicht
  got_cmd    = true;
  last_cmd_ms = millis();

  // Merke Zeitpunkt, wann Ball WIRKLICH gesehen wurde
  if (last_valid > 0.5f) {
    last_ball_ms = millis();
  }

  // ACK
  uint8_t ack_body[2] = { 0xA1, pkt.seq };
  uint32_t ack_crc = CRC32.crc32(ack_body, sizeof(ack_body));
  uint8_t ack_frame[6];
  memcpy(&ack_frame[0], ack_body, 2);
  memcpy(&ack_frame[2], &ack_crc, 4);
  cobsSerial.send(ack_frame, sizeof(ack_frame));
}

// ═══════════════════════════════════════════════════════════════
//                         SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
   Serial.begin(2000000);
  cobsSerial.setStream(&Serial);
  cobsSerial.setPacketHandler(&onPacketReceived);

  Wire1.begin();
  gyro.begin();
  LidarBegin();


  // pinMode(kickerPin, OUTPUT);
  // digitalWrite(kickerPin, LOW);

  // delay(1000);

  // digitalWrite(kickerPin, HIGH);
  // delay(20);
  // digitalWrite(kickerPin, LOW);




  state = NO_BALL;

  delay(1000);
}



static Vec2 computeBehindBallTarget(float ballX, float ballY) {
  Vec2 offset = {0.0f, 0.0f};
  // Einfache Logik: Hinter den Ball fahren
  if (ballY > 0.0f) {
    if (fabsf(ballX) < 10.0f) {
      offset.x = 0.0f; offset.y = 0.0f;
    } else {
      offset.y = 80.0f;
    }
  } else {
    offset.x = (ballX < 0.0f) ? -50.0f : 50.0f;
    offset.y = 75.0f;
  }
  return { ballX - offset.x, ballY - offset.y };
}








// ═══════════════════════════════════════════════════════════════
//                          LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {

  // ──────────── 0.  Sensor-Updates ────────────────────────────
  cobsSerial.update();
  lidaar();
  gyro.update();

  g_a = gyro.getAngleDegrees();
  p_x = Player.x * 0.1f;        // mm → cm
  p_y = Player.y * 0.1f;

  const uint32_t now = millis();

  // ──────────── 1.  Heading-PID (immer aktiv) ────────────────
  float rotCmd = pidGyro.update(g_a);

  // ──────────── 2.  Ball-Status bestimmen ────────────────────
  bool ballVisible = (last_valid > 0.5f)
                  && (now - last_cmd_ms < CMD_TIMEOUT_MS);

  //  Hysterese:  Ball erst nach BALL_LOST_TIMEOUT_MS als "verloren" werten
  //              → verhindert Flackern bei kurzen Verdeckungen
  bool ballRecent = (now - last_ball_ms) < BALL_LOST_TIMEOUT_MS;

  // ──────────── 3.  State-Übergänge ─────────────────────────
  switch (state) {

    case NO_BALL:
      if (ballVisible) {
        state = CHASE_BALL;
        pidBallX.reset();          // kein Integral-Altlast
        pidBallY.reset();
      }
      break;

    case CHASE_BALL:
      if (!ballVisible && !ballRecent) {
        state = NO_BALL;
        pidCenterX.reset();
        pidCenterY.reset();
      }
      break;
  }

  // ──────────── 4.  Fahrbefehl berechnen ─────────────────────
  Vec2 driveCmd = {0.0f, 0.0f};

  switch (state) {

    // ── NO_BALL:  Zurück zur Feldmitte ───────────────────────
    case NO_BALL: {
      //  Fehler = Position (NICHT negiert – passt zu eurem Drive/Lidar-System)
      float errX = p_x;
      float errY = p_y;

      driveCmd.x = pidCenterX.update(errX);
      driveCmd.y = pidCenterY.update(errY);

      //  Speed deckeln (zur Mitte nicht rasen)
      // constexpr float MAX_CENTER_SPEED = 120.0f;
      // float mag = sqrtf(driveCmd.x * driveCmd.x + driveCmd.y * driveCmd.y);
      // if (mag > MAX_CENTER_SPEED) {
      //   float s = MAX_CENTER_SPEED / mag;
      //   driveCmd.x *= s;
      //   driveCmd.y *= s;
      // }
      break;
    }

    // ── CHASE_BALL:  Zum Ball fahren ─────────────────────────
     case CHASE_BALL: {

      //  Ball in lokalen Pixel-Koordinaten (Spiegelzentrum)
      float bx = -(last_vx - CAM_CENTER_X);    // -  = Spiegel invertiert X
      float by = -(last_vy - CAM_CENTER_Y);     // +  = Ball vorne

   Vec2 v = computeBehindBallTarget(bx, by);


      

      //  PID → Fahrbefehle
      driveCmd.x = pidBallX.update(v.x);
      driveCmd.y = pidBallY.update(v.y);

      // Debug

      Serial.print(">ball_bx:");      Serial.println(bx);
      Serial.print(">ball_by:");      Serial.println(by);
      break;
    }
  }

  // ──────────── 5.  Out-of-Bounds: Axis-Lock + Pull ───────────
   applyFieldBounds(driveCmd, p_x, p_y, kBounds);

  // ──────────── 6.  Notfall: kein Pi-Kontakt ────────────────
  // if (!got_cmd || (now - slast_cmd_ms) > CMD_TIMEOUT_MS) {
  //   driveCmd.x *= 0.3f;          // Sanft ausbremsen
  //   driveCmd.y *= 0.3f;
  // }

  // ──────────── 7.  Motoren ansteuern ────────────────────────
  Drive.calcDrive(driveCmd.x, -driveCmd.y, -rotCmd);
  Drive.drive();


  // ──────────── 8.  Debug / Teleplot ─────────────────────────
  // Serial.print(">state:");       Serial.println(state);
  // Serial.print(">p_x:");        Serial.println(p_x);
  // Serial.print(">p_y:");        Serial.println(p_y);
  // Serial.print(">ball_valid:"); Serial.println(last_valid);
  // Serial.print(">drv_x:");      Serial.println(driveCmd.x);
  // Serial.print(">drv_y:");      Serial.println(driveCmd.y);
  // Serial.print(">rotcmd:");      Serial.println(rotCmd);
}

