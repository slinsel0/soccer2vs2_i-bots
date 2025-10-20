#include <Arduino.h>
#include "DriveSystem.h"
#include "LD19.h" 
#include "LidarProcessing.h"
#include <Wire.h>
#include "GyroSystem.h"
#include <PacketSerial.h>   // by Christopher Baker
#include <FastCRC.h>        // by Frank Boesing
#include <PIDController.h>
#include <math.h>

COBSPacketSerial cobsSerial;
FastCRC32 CRC32;
Adafruit_BNO055 bno(55, 0x28); // 0x28 oder 0x29, je nach ADR-Pin

GyroSystem gyro;

DriveSystem Drive;

PIDController pidg(0.45f,  0.0f,   50.0f, 1.5f,  0.0f);
PIDController pidx(1.725f, 0.0f, 20.0f, 1.5f, 0.0f);
PIDController pidy(1.725f, 0.0, 20.0f, 1.5f, 0.0f);




float g_a =0;
float p_x= 0;
float p_y= 0;
float pdg = 0.0f;
float fp_x =0.0f;
float fp_y =0.0f;




struct __attribute__((packed)) VectorCmd {
  uint8_t  msg_id;   // 1 = velocity command
  uint8_t  seq;
  uint32_t t_us;     // Timestamp vom Sender (optional)
  float    vx;
  float    vy;
  float    omega;
  uint32_t crc32;    // CRC über alle Bytes VOR diesem Feld
};
static_assert(sizeof(VectorCmd) == 22, "VectorCmd must be 22 bytes");

volatile float    last_vx = 0, last_vy = 0, last_omega = 0;
volatile uint8_t  last_seq = 0;
volatile uint32_t last_t_us = 0;
volatile bool     got_cmd = false;
volatile uint32_t last_cmd_ms = 0;

constexpr uint32_t CMD_TIMEOUT_MS = 200; // reset quickly when command stream stalls

constexpr float CM_TO_M = 0.01f;
constexpr float M_TO_MM = 100.0f;
constexpr float DEFAULT_SAFETY_DT = 0.0005f; // 5 ms fallback for safety controller

void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < sizeof(VectorCmd)) return;

  // CRC prüfen (über alles außer die letzten 4 CRC-Bytes)
  uint32_t calc = CRC32.crc32(buffer, sizeof(VectorCmd) - 4);

  VectorCmd pkt;
  memcpy(&pkt, buffer, sizeof(VectorCmd)); // little-endian → Teensy (ARM) passt

  if (calc != pkt.crc32) {
    // CRC-Fehler -> ignorieren
    return;
  }
  if (pkt.msg_id != 1) {
    // andere msg-typen ggf. hier behandeln
    return;
  }

  last_vx   = pkt.vx;
  last_vy   = pkt.vy;
  last_omega= pkt.omega;
  last_seq  = pkt.seq;
  last_t_us = pkt.t_us;
  got_cmd   = true;
  last_cmd_ms = millis();

  // (Optional) kleines Ack zurücksenden (gleiches Framing: COBS + CRC32)
  // Hier schicken wir msg_id=0xA1, seq zurück.
  uint8_t ack_body[2] = { 0xA1, pkt.seq };
  uint32_t ack_crc = CRC32.crc32(ack_body, sizeof(ack_body));
  uint8_t ack_frame[2 + 4];
  memcpy(&ack_frame[0], ack_body, 2);
  memcpy(&ack_frame[2], &ack_crc, 4);
  cobsSerial.send(ack_frame, sizeof(ack_frame)); // COBS + 0x00 übernimmt die Lib
}







static Vec2 computeBehindBallTarget(float ballX, float ballY) {
  Vec2 offset = {0.0f, 0.0f};

  if (ballY > 0.0f) {
    if (fabsf(ballX) < 15.0f) {
      // Ball mittig vorne: keine Versetzung notwendig

     offset.x = 0.0f;
      offset.y = 0.0f;

    } else {
      offset.y = 70.0f;
    }
  } else {
    if (ballX < 0.0f) {
      offset.x = -55.0f;
      offset.y = 30.0f;
    } else {
      offset.x = 55.0f;
      offset.y = 30.0f;
    }
  }

  Vec2 target;
  target.x = ballX - offset.x;
  target.y = ballY - offset.y;
  return target;
}






constexpr float PLAYING_FIELD_WIDTH_CM  = 140.0f;
constexpr float PLAYING_FIELD_LENGTH_CM = 208.0f;
constexpr float OUTER_AREA_CM           = 12.0f;  // not used directly yet, kept for clarity
constexpr float GOAL_WIDTH_CM           = 60.0f;
constexpr float PENALTY_AREA_DEPTH_CM   = 80.0f;
constexpr float SAFETY_MARGIN_TO_LINE   = 1.0f;
constexpr float SAFETY_BLEND_DISTANCE   = 1.0f;

// Simple guard that keeps the robot from remaining inside the goal corridors.
// It overrides the commanded translation when we are within the penalty depth in
// front of a goal (based on the official field dimensions) and gradually locks
// the lateral axis while pushing back toward the field center.
static Vec2 applyGoalBoundaryGuard(const Vec2& desiredCmd, Vec2 robotPosCm) {
  Vec2 adjusted = desiredCmd;

  const float fieldHalfY         = PLAYING_FIELD_LENGTH_CM * 0.5f;
  const float penaltyEntryY      = fieldHalfY - PENALTY_AREA_DEPTH_CM;
  const float lockStartY         = fieldHalfY - SAFETY_MARGIN_TO_LINE;
  const float blendStartY        = fieldHalfY - SAFETY_BLEND_DISTANCE;
  const float corridorHalfWidth  = (GOAL_WIDTH_CM * 0.5f) ;
  const float driveLimit         = static_cast<float>(maxSpeed);

  const float absY   = fabsf(robotPosCm.y);
  const float absX   = fabsf(robotPosCm.x);
  const float signY  = (robotPosCm.y >= 0.0f) ? 1.0f : -1.0f;

  if (absY >= penaltyEntryY) {
    const float zoneSpan      = fmaxf(fieldHalfY - penaltyEntryY, 1.0f);
    const float depthFraction = constrain((absY - penaltyEntryY) / zoneSpan, 0.0f, 1.0f);
    const float pushMin       = 0.35f * driveLimit;           // soft pull when just entering the area
    const float pushStrength  = pushMin + depthFraction * (driveLimit - pushMin);

    adjusted.y = -signY * pushStrength;

    if (absX <= corridorHalfWidth) {
      if (absY >= blendStartY && blendStartY < lockStartY) {
        const float denom      = fmaxf(lockStartY - blendStartY, 1.0f);
        const float lockFactor = constrain((absY - blendStartY) / denom, 0.0f, 1.0f);
        adjusted.x *= (1.0f - lockFactor);
      }
      if (absY >= lockStartY) {
        adjusted.x = 0.0f;
      }
    } else {
      adjusted.x = constrain(adjusted.x, -0.4f * driveLimit, 0.4f * driveLimit);
    }
  }

  if (absY >= lockStartY) {
    const float overshoot = absY - lockStartY;
    if (overshoot > 0.0f) {
      const float required = constrain(overshoot * 12.0f, 0.0f, driveLimit);
      const float current  = fabsf(adjusted.y);
      adjusted.y = -signY * fmaxf(current, required);
    }
  }

  adjusted.x = constrain(adjusted.x, -driveLimit, driveLimit);
  adjusted.y = constrain(adjusted.y, -driveLimit, driveLimit);
  return adjusted;
}





void setup() {
      gyro.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(2000000);
  cobsSerial.setStream(&Serial);
  cobsSerial.setPacketHandler(&onPacketReceived);
  Wire1.begin();
    gyro.begin();
    
  LidarBegin();

}







uint32_t lastBlink = 0;
uint32_t lastSafetyUpdateUs = 0;

void loop() {
  cobsSerial.update(); // wichtig: ruft intern den Decoder auf

  lidaar();   // Funktion für LiDAR-Daten
  gyro.update();
  g_a = gyro.getAngleDegrees();

  float pdg = pidg.updatePD(g_a); // pdg bleibt wahrscheinlich float

  p_x = Player.x;
  p_y = Player.y;

  Serial.print('x');
  Serial.println(p_x);
    Serial.print('y');
  Serial.println(p_y);

  Vec2 v = computeBehindBallTarget(last_vx, last_vy);



  float finalbvx = pidx.update(v.x);
  float finalbvy = pidy.update(v.y);

  uint32_t nowMs = millis();
  if (got_cmd && (nowMs - last_cmd_ms) > CMD_TIMEOUT_MS) {
    got_cmd = false;
  }

  Vec2 ball = { finalbvx, finalbvy };

  if (!got_cmd) {
    ball.x = 0.0f;
    ball.y = 0.0f;
  }

  ball = applyGoalBoundaryGuard(ball, Player);

  Drive.calcDrive(-ball.x, -ball.y, -pdg);

  Drive.drive();
}
