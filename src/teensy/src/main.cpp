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
#include <outofbounce.h>

COBSPacketSerial cobsSerial;
FastCRC32 CRC32;
Adafruit_BNO055 bno(55, 0x28); // 0x28 oder 0x29, je nach ADR-Pin

GyroSystem gyro;

DriveSystem Drive;

PIDController pidg(5.25f,  0.00001f,   2.50f, 1.5f,  0.0f);
PIDController pidx(2.60, 0.0f, 10.0f, 1.5f, 0.0f);
PIDController pidy(2.6, 0.0, 10.0f, 1.5f, 0.0f);

static const BoundsConfig kBounds = {
  /* xLimit     */ 80.0f,    // halbe kurze Seite
  /* yLimit     */ 100.5f,   // halbe lange Seite
  /* softMargin */ 26.0f,    // 20 cm vor Linie: Soft-Bremse
  /* hardMargin */  18.5f,    // 8 cm vor Linie: Hard-Pushback
  /* kPush      */  0.85f,    // mittlerer Rückstoß
  /* maxSoft    */ 70.0f,    // max Speed in Soft-Zone
  /* maxHard    */ 46.0f     // max Speed in Hard-Zone
};
static const BoundsExtras kExtras = {
  /* yEscapeThresh           */ 20.0f,  // "hinter/vor mir" in lokalen cm
  /* xEscapeThresh           */ 30.0f,  // "deutlich andere Seite" in lokalen cm
  /* enableDirectionalEscape */ false
};


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
    if (fabsf(ballX) < 10.0f) {
      // Ball mittig vorne: keine Versetzung notwendig

     offset.x = 0.0f;
      offset.y = 0.0f;

    } else {
      offset.y = 70.0f;
    }
  } else {
    if (ballX < 0.0f) {
      offset.x = -35.0f;
      offset.y = 75.0f;
    } else {
      offset.x = 35.0f;
      offset.y = 75.0f;
    }
  }

  Vec2 target;
  target.x = ballX - offset.x;
  target.y = ballY - offset.y;
  return target;
}









void setup() {
      gyro.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(2000000);
  cobsSerial.setStream(&Serial);
  cobsSerial.setPacketHandler(&onPacketReceived);
  Wire1.begin();
    
  LidarBegin();

}







uint32_t lastBlink = 0;
uint32_t lastSafetyUpdateUs = 0;

void loop() {
  cobsSerial.update(); // wichtig: ruft intern den Decoder auf

  lidaar();   // Funktion für LiDAR-Daten
  gyro.update();
  g_a = gyro.getAngleDegrees();
  float bs = gyro.getAngleRadians(); // pdg bleibt wahrscheinlich float


  float pdg = pidg.updatePD(g_a); // pdg bleibt wahrscheinlich float


  p_x = Player.x;
  p_y = Player.y;








  Vec2 v = computeBehindBallTarget(last_vx, last_vy);

  
  float finalbvx = pidx.update(v.x);
  float finalbvy = pidy.update(v.y);


  Vec2 ball = { finalbvx, finalbvy };
  Vec2 ballLocal = { last_vx, last_vy };

  applyFieldBounds(ball, p_x, p_y, kBounds, ballLocal);



  uint32_t nowMs = millis();
  if (got_cmd && (nowMs - last_cmd_ms) > CMD_TIMEOUT_MS) {
    got_cmd = false;
  }


  if (!got_cmd) {
    ball.x = 0.0f;
    ball.y = 0.0f;
  }






  Drive.calcDrive(ball.x, -ball.y, -pdg);

  Serial.println(ballLocal.x);

  Drive.drive();

}
