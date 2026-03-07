
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
#include "Rotationauthority .h"

COBSPacketSerial cobsSerial;
FastCRC32        CRC32;
GyroSystem       gyro;
DriveSystem      Drive;
RotationAuthority rotAuth;    // Fuzzy Heading-vs-Translation


// ═══════════════════════ STATE MACHINE ══════════════════════════
enum RobotState {
  NO_BALL,        // Ball nicht sichtbar  → zurück zur Mitte
  CHASE_BALL,     // Ball sichtbar        → hinfahren
  DRIVE_TO_GOAL   // Ball in Kuhle        → zum Tor fahren
};

static RobotState state = NO_BALL;

// ─────────────────── Zeitkonstanten (ms) ───────────────────────
static constexpr uint32_t BALL_LOST_TIMEOUT_MS  = 800;   // nach 500 ms ohne Ball → NO_BALL
static constexpr uint32_t CMD_TIMEOUT_MS        = 300;   // Kein Paket vom Pi     → Notfall

// ─────────────────── IR-Sensor Debounce ────────────────────────
static constexpr uint32_t IR_DEBOUNCE_LOW_MS    = 80;    // IR muss 80 ms  LOW  → Ball sicher in Kuhle
static constexpr uint32_t IR_DEBOUNCE_HIGH_MS   = 350;   // IR muss 150 ms HIGH → Ball sicher raus

// ─────────────────── Tor-Zielkoordinaten (cm, Lidar) ──────────
static constexpr float    GOAL_TARGET_Y         = 82.0f; // Y-Ziel: kurz vor Torlinie (+Y = gegn. Tor)

// ═══════════════════ PID-REGLER ════════════════════════════════

PIDController pidGyro(0.85f, 0.000f, 0.32f, /* dt_ms */ 1, /* iLim */ 25.0f);


PIDController pidBallX(0.595f, 0.001f, 0.0f, 2, 28.0f);
PIDController pidBallY(0.595f, 0.001f, 0.0f, 2, 28.0f);

PIDController pidCenterX(1.5f, 0.0f, 0.2f, 2, 0.0f);
PIDController pidCenterY(1.5f, 0.0f, 0.2f, 2, 0.0f);


static const BoundsConfig kBounds = {
  // ── Feldgrenzen (halbe Maße in cm) ──
  /* xLimit          */  65.0f,    // 182 / 2
  /* yLimit          */ 103.5f,    // 243 / 2

  // ── Tor-Geometrie ──
  /* goalHalfWidth   */  30.0f,    // 60 / 2

  // ── Sicherheitsmargen ──
  /* safeMarginX     */  3.0f,    // safeLine_X = 91 - 15 = 76 cm
  /* safeMarginY     */  4.0f,    // safeLine_Y (Wand) = 121.5 - 10 = 111.5 cm
  /* goalSafeMarginY */  25.0f,    // safeLine_Y (Tor)  = 121.5 - 30 = 91.5 cm


  // ── Pull-Regler ──
  /* kPull           */  2.0f,  
  /* maxPull         */ 80.0f     
};
// ═══════════════════ KAMERA-KONSTANTEN ═════════════════════════
// offset für kamera guck in src/cam/config.json
static constexpr float CAM_CENTER_X = 718.0f;
static constexpr float CAM_CENTER_Y = 595.0f;

// ═══════════════════ GLOBALE VARIABLEN ═════════════════════════
float g_a = 0;           // Gyro angeldegess ausrichtung
float p_x = 0;           // Roboter-Position X in cm (Lidar)
float p_y = 0;           // Roboter-Position Y in cm (Lidar)
const int kickerPin  = 22;  // Ausgang zur Spule
const int triggerPin = 13;  // Eingang
const int ir1_sensor = 23;   // Eingang

// Kicker status Variablen
static uint32_t kicker_condition_start_ms = 0;
static uint32_t last_kick_time_ms = 0;
static bool     is_kicking = false;

// IR-Sensor Debounce Variablen
static uint32_t ir_low_since      = 0;
static uint32_t ir_high_since     = 0;
static bool     ir_debounced_low  = false;


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
volatile float    last_valid    = 0;     
volatile bool     got_cmd       = false;
volatile uint32_t last_cmd_ms   = 0;
volatile uint32_t last_ball_ms  = 0;     

// ─────────────────── Paket-Handler (ISR-safe) ──────────────────
void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < sizeof(VectorCmd)) return;

  uint32_t calc = CRC32.crc32(buffer, sizeof(VectorCmd) - 4);
  VectorCmd pkt;
  memcpy(&pkt, buffer, sizeof(VectorCmd));

  if (calc != pkt.crc32 || pkt.msg_id != 1) return;


  last_valid = pkt.omega;
  if((last_valid > 0.5f) || millis() > (last_ball_ms + 1000)){
    last_vx    = pkt.vx;
    last_vy    = pkt.vy;       // 1.0 = Ball da, 0.0 = nicht
  }

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


void setup() {
 Serial.begin(2000000);
  cobsSerial.setStream(&Serial);
  cobsSerial.setPacketHandler(&onPacketReceived);

  gyro.begin();
  LidarBegin();

  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(ir1_sensor, INPUT);

pinMode(kickerPin, OUTPUT);

digitalWrite(kickerPin, LOW);






  delay(1000);





  state = NO_BALL;

}



static Vec2 computeBehindBallTarget(float ballX, float ballY) {
  Vec2 offset = {0.0f, 0.0f};
  
  if (ballY > 0.0f) { // wenn der ball vorne ist
    if (fabsf(ballX) < 100.0f) {
      offset.x = 0.0f; offset.y = 0.0f;
    } else {
      offset.y = 200.0f; // offset damit er nicht direkt drafu fährt 
    }
  } else { // wenn der ball hinten ist 
    offset.x = (ballX < 0.0f) ? -180.0f : 180.0f; 
    offset.y =100.0f;
  }
  return { ballX - offset.x, ballY - offset.y };
}








// ═══════════════════════════════════════════════════════════════
//                          LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {

  // ──────────── 0.  Sensor-Updates ────────────────────────────
  cobsSerial.update(); // serielle kommunkation between pi 5 and teensy 4.1
  lidaar();  // Verarbeitung plus Kommunkation
  gyro.update(); 

  g_a = gyro.getAngleDegrees();
  float g_g = gyro.getAngleRadians();
  p_x = Player.x * 0.1f;        // mm → cm
  p_y = Player.y * 0.1f;

  const uint32_t now = millis();

  //float rotCmd = pidGyro.update(g_a);


  float rotCmd = 0;




      // Serial.print(">g_a:");    
      
      // Serial.println(g_a);

      // Serial.print(">rotCmd:");    
      
      // Serial.println(rotCmd);






  bool ballVisible = (last_valid > 0.5f)
                  && (now - last_cmd_ms < CMD_TIMEOUT_MS);

  bool ballRecent = (now - last_ball_ms) < BALL_LOST_TIMEOUT_MS;

  // ──────────── 2b. IR-Sensor Debounce ──────────────────────────
  bool ir_raw_low = (digitalRead(ir1_sensor) == LOW);
  if (ir_raw_low) {
    ir_high_since = 0;
    if (ir_low_since == 0) ir_low_since = now;
    if ((now - ir_low_since) >= IR_DEBOUNCE_LOW_MS) ir_debounced_low = true;
  } else {
    ir_low_since = 0;
    if (ir_high_since == 0) ir_high_since = now;
    if ((now - ir_high_since) >= IR_DEBOUNCE_HIGH_MS) ir_debounced_low = false;
  }

  // ──────────── 3.  State-Übergänge ─────────────────────────
  switch (state) {

    case NO_BALL:
      if (ir_debounced_low) {
        // Ball in Kuhle aber Kamera sieht nichts → direkt zum Tor
        state = DRIVE_TO_GOAL;
        pidCenterX.reset();
        pidCenterY.reset();
      } else if (ballVisible) {
        state = CHASE_BALL;
        pidBallX.reset();          // kein Integral-Altlast
        pidBallY.reset();
      }
      break;

    case CHASE_BALL:
      if (ir_debounced_low) {
        // Ball ist in der Kuhle → zum Tor fahren
        state = DRIVE_TO_GOAL;
        pidCenterX.reset();
        pidCenterY.reset();
      } else if (!ballVisible && !ballRecent) {
        state = NO_BALL;
        pidCenterX.reset();
        pidCenterY.reset();
      }
      break;

    case DRIVE_TO_GOAL:
      if (!ir_debounced_low) {
        // Ball ist raus → Kamera übernimmt wieder
        state = CHASE_BALL;
        pidBallX.reset();
        pidBallY.reset();
       } 
      //  else if (p_y >= GOAL_TARGET_Y &&
      //            fabsf(p_y - GOAL_TARGET_Y) < GOAL_ARRIVED_DIST) {
      //   // Am Tor angekommen → Ball abgegeben
      //   state = NO_BALL;
      //   pidCenterX.reset();
      //   pidCenterY.reset();
      // }
      break;
  }

  // ──────────── 4.  Fahrbefehl berechnen ─────────────────────
  Vec2 driveCmd = {0.0f, 0.0f};

  switch (state) {

    case NO_BALL: {
      float errX = -p_x;
      float errY = -p_y;

      driveCmd.x = pidCenterX.update(errX);
      driveCmd.y = pidCenterY.update(errY);

       rotCmd = pidGyro.update(g_a);

      break;
    }

    case CHASE_BALL: {
      float bx = (last_vx - CAM_CENTER_X);
      float by = (last_vy - CAM_CENTER_Y);

      Vec2 v = computeBehindBallTarget(-bx, -by);

      driveCmd.x = pidBallX.update(-v.x);
      driveCmd.y = pidBallY.update(-v.y);

      rotCmd = pidGyro.update(g_a);


      break;
    }

    case DRIVE_TO_GOAL: {


      // X → Feldmitte (zentriert aufs Tor zufahren)
      // Y → Richtung gegnerisches Tor
      float errX = 0.0f - p_x;
      float errY = GOAL_TARGET_Y - p_y;   

      driveCmd.x = pidCenterX.update(errX*1.5f);
      driveCmd.y = pidCenterY.update(-errY);

      // Zielwinkel in Grad (0° = +Y Richtung, positiv = rechts)
      float goalAngleDeg = atan2f(errX, errY) * (180.0f / PI);
      // Heading-Fehler: Differenz zwischen Soll und Ist
      float headingErr = goalAngleDeg - g_a;
 
       rotCmd = pidGyro.update(-headingErr);


      break;
    }
  }

    applyFieldBounds(driveCmd, p_x, p_y, kBounds);

  // float transMag = sqrtf(driveCmd.x * driveCmd.x + driveCmd.y * driveCmd.y);
  // float scaledRot = rotAuth.apply(goalRotCmd, g_a, transMag); 

  if (digitalRead(triggerPin) == HIGH) {

      int motorVRpin[3] = {3, 4, 15}; 
  int motorHRpin[3] = {5, 6, 19}; 
  int motorHLpin[3] = {10, 7, 14};
  int motorVLpin[3] = {12, 11, 18};
    Drive.setMotor(motorVRpin[0], motorVRpin[1], motorVRpin[2], 255);
    Drive.setMotor(motorHRpin[0], motorHRpin[1], motorHRpin[2], 255);
    Drive.setMotor(motorHLpin[0], motorHLpin[1], motorHLpin[2], 255);
    Drive.setMotor(motorVLpin[0], motorVLpin[1], motorVLpin[2], 255);



 
  }

  else {

    if (state == DRIVE_TO_GOAL){

          Drive.calcDriveRotation(driveCmd.x, driveCmd.y, -rotCmd, -g_g);

    }
    else {
    Drive.calcDrive(driveCmd.x, driveCmd.y, -rotCmd);
    }
        Drive.drive();


    // Drive.calcDriveRotation(driveCmd.x, driveCmd.y, -scaledRot,);
  }


  // ──────────── 7.  Kicker Logik ─────────────────────────────
  // float kick_ballX = CAM_CENTER_X - last_vx;  
  // float kick_ballY = CAM_CENTER_Y - last_vy;  

  // if (state == CHASE_BALL && ballVisible && fabsf(kick_ballX) < 50.0f && kick_ballY > 5.0f) {
  //   if (kicker_condition_start_ms == 0) {
  //     kicker_condition_start_ms = now;
  //   }
  // } else {
  //   kicker_condition_start_ms = 0;
  // }

  // // Kicker nach 250ms auslösen (max. jede Sekunde)
  // if (kicker_condition_start_ms != 0 && (now - kicker_condition_start_ms) >= 250) {
  //   if (!is_kicking && (last_kick_time_ms == 0 || (now - last_kick_time_ms) >= 5000)) {
  //     is_kicking = true;
  //     last_kick_time_ms = now;
  //     digitalWrite(kickerPin, HIGH); // Schiessen (Gate an)
  //   }
  // }

  // // Schutz für das Gate: Maximal 10 ms schießen
  // if (is_kicking && (now - last_kick_time_ms) >= 10) {
  //   is_kicking = false;
  //   digitalWrite(kickerPin, LOW); 
  // }

  // ──────────── 8.  Debug / Teleplot ─────────────────────────
  Serial.print(">state:");       Serial.println(state);
  Serial.print(">p_x:");        Serial.println(p_x);
  Serial.print(">p_y:");        Serial.println(p_y);
  // Serial.print(">ball_valid:"); Serial.println(last_valid);
  // Serial.print(">MOTORVR:");      Serial.println(Drive.getMotorVR());
  // Serial.print(">MOTORHR:");      Serial.println(Drive.getMotorHR());
  // Serial.print(">MOTORHL:");      Serial.println(Drive.getMotorHL());
  // Serial.print(">MOTORVL:");      Serial.println(Drive.getMotorVL());
  // Serial.print(">rotcmd:");      Serial.println(rotCmd);





}


