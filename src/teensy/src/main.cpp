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
#include <keeper.h>

COBSPacketSerial cobsSerial;
FastCRC32 CRC32;

GyroSystem gyro;
DriveSystem Drive;

// --- TUNING ---
// Gyro: P etwas runter, D fast weg -> Verhindert "Zittern", das Speed frisst
PIDController pidg(6.5f,  0.001f, 0.1f, 0.0f, 25.0f); 
PIDController pidb(6.5f,  0.001f, 0.1f, 0.0f, 25.0f);

// Ball X/Y: D=0.0 ist WICHTIG! D=2.0 hat vorher extrem gebremst.
// P=2.5 bedeutet: Bei 100px Fehler (Kamera) -> 250 Speed (Vollgas).
PIDController pidx(2.5f, 0.0f, 0.0f, 0.0f, 0.0f);
PIDController pidy(2.5f, 0.0f, 0.0f, 0.0f, 0.0f);

// --- GRENZEN ---
static const BoundsConfig kBounds = {
  /* xLimit      */ 80.0f,    // Fast an der Wand (91)
  /* yLimit      */ 100.0f,   // Fast an der Wand (121.5)
  
  /* softMargin  */ 35.0f,    // Bremsen beginnt bei X=55 / Y=85 (Viel Platz!)
  /* hardMargin  */ 12.0f,    // Rückstoß beginnt bei X=78 / Y=108 (Sicherheitsabstand)
  
  /* kPush       */  1.0f,    // Kräftiger Rückstoß
  /* maxSoft     */ 100.0f,   // Erlaube höhere Geschwindigkeit in der Randzone
  /* maxHard     */ 50.0f     
};

static const BoundsExtras kExtras = {
  /* yEscapeThresh           */ 20.0f,  
  /* xEscapeThresh           */ 30.0f,  
  /* enableDirectionalEscape */ true 
};

float g_a = 0;
float p_x = 0;
float p_y = 0;
float finaldrivex = 0.0f;
float finaldrivey = 0.0f;

// --- COMMS ---
struct __attribute__((packed)) VectorCmd {
  uint8_t  msg_id;   
  uint8_t  seq;
  uint32_t t_us;     
  float    vx;
  float    vy;
  float    omega;
  uint32_t crc32;    
};

volatile float    last_vx = 0, last_vy = 0, last_omega = 0;
volatile bool     got_cmd = false;
volatile uint32_t last_cmd_ms = 0;
constexpr uint32_t CMD_TIMEOUT_MS = 200; 

void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < sizeof(VectorCmd)) return;
  uint32_t calc = CRC32.crc32(buffer, sizeof(VectorCmd) - 4);
  VectorCmd pkt;
  memcpy(&pkt, buffer, sizeof(VectorCmd)); 

  if (calc != pkt.crc32 || pkt.msg_id != 1) return;

  last_vx   = pkt.vx;
  last_vy   = pkt.vy;
  last_omega= pkt.omega;
  got_cmd   = true;
  last_cmd_ms = millis();

  // Ack
  uint8_t ack_body[2] = { 0xA1, pkt.seq };
  uint32_t ack_crc = CRC32.crc32(ack_body, sizeof(ack_body));
  uint8_t ack_frame[6];
  memcpy(&ack_frame[0], ack_body, 2);
  memcpy(&ack_frame[2], &ack_crc, 4);
  cobsSerial.send(ack_frame, sizeof(ack_frame)); 
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

void setup() {
  gyro.begin(); // IMUPLUS Mode beachten (siehe vorherige Antwort)
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(2000000);
  cobsSerial.setStream(&Serial);
  cobsSerial.setPacketHandler(&onPacketReceived);
  Wire1.begin();
  LidarBegin();
}

void loop() {
  cobsSerial.update(); 
  lidaar();   
  gyro.update();
  
  g_a = gyro.getAngleDegrees();
  p_x = Player.x;
  p_y = Player.y;

  // 1. Rotation berechnen (Heading halten 0°)
  float pdg = pidg.update(g_a);

  // 2. Ball Position holen
  Vec2 ballLocal = { last_vx, last_vy };
  ballLocal.x += 25; // Kamera Offset?

  // Keeper Logik (optional)
  keeper(Player, ballLocal);

  // 3. Ziel berechnen & PID anwenden
  Vec2 v = computeBehindBallTarget(ballLocal.x, ballLocal.y);
  float finalbvx = pidx.update(v.x);
  float finalbvy = pidy.update(v.y);
  Vec2 ballVec = { finalbvx, finalbvy };

  // 4. Feldgrenzen anwenden
  applyFieldBounds(ballVec, p_x, p_y, kBounds, ballLocal, kExtras);

  finaldrivex = ballVec.x;
  finaldrivey = ballVec.y;

  // Timeout Safety
  uint32_t nowMs = millis();
  if (!got_cmd || (nowMs - last_cmd_ms) > CMD_TIMEOUT_MS) {
    got_cmd = false;
    // Rückkehr zur Mitte wenn Ball verloren? 
    // Vorsichtig: -p_x * 2 ist sehr aggressiv! Lieber stehen bleiben oder sanft bremsen.
    finaldrivex = -p_x * 2; 
    finaldrivey = -p_y * 2;
  }

  // 5. Fahren
  Drive.calcDrive(finaldrivex, -finaldrivey, -pdg);
  Drive.drive();
}