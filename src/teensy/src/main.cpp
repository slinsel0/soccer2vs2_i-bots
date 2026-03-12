
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
#include "keeper.h"
#include <Servo.h>

Servo dribbler;
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

PIDController pidGyro(0.85f, 0.000f, 0.22f, /* dt_ms */ 1, /* iLim */ 25.0f);


PIDController pidBallX(0.6f, 0.004f, 0.02f, 2, 28.0f);
PIDController pidBallY(0.6f, 0.004f, 0.02f, 2, 28.0f);

PIDController pidCenterX(1.8f, 0.0f, 0.2f, 2, 0.0f);
PIDController pidCenterY(1.8f, 0.0f, 0.2f, 2, 0.0f);




static const BoundsConfig kBounds = {
  // ── Feldgrenzen (halbe Maße in cm) ──
  /* xLimit          */  70.0f,
  /* yLimit          */ 95.5f,

  // ── Tor-Geometrie ──
  /* goalHalfWidth   */  30.0f,

  // ── Sicherheitsmargen ──
  /* safeMarginX     */  5.0f,    
  /* safeMarginY     */  8.0f,    
  /* goalSafeMarginY */  22.0f,    

  // ── Damping & Return Parameter ──
  /* dampingMargin   */  25.0f,    // 10 cm vor der Linie beginnt das Damping
  /* returnKp        */   4.0f     // Stärke des Zurück-Schiebens ins Feld
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

 int dribblerPin = 2;
// Kicker status Variablen
static uint32_t last_kick_time_ms = 0;
static bool     is_kicking = false;
static uint32_t kick_start_ms    = 0;

// Kicker Konstanten
static constexpr uint32_t KICK_GATE_MS       = 10;    // Gate max 10 ms offen
static constexpr uint32_t KICK_COOLDOWN_MS   = 1000;  // nur 1× pro Sekunde kicken
static constexpr float    KICK_BALL_THRESH   = 400.0f; // |bx|<50 && |by|<50 → Ball in Kuhle

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

// digitalWrite(kickerPin, HIGH);
// delay(15);
// digitalWrite(kickerPin, LOW);

// dribbler.attach(2);

//   // ESC initial auf Minimum setzen
//   dribbler.writeMicroseconds(1000);
//    delay(3000);  // Zeit zum Armen / Initialisieren
//     delay(3000);

//     dribbler.writeMicroseconds(1145);
//     delay(3000);
  

  // delay(2000);

  // // wieder runter
  // for (int pwm = 1600; pwm >= 1000; pwm -= 10) {
  //   dribbler.writeMicroseconds(pwm);
  //   delay(50);
  // }

  // delay(2000);






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



    if (digitalRead(triggerPin) == LOW) {


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
  // bool ir_raw_low = (digitalRead(ir1_sensor) == LOW);
  // if (ir_raw_low) {
  //   ir_high_since = 0;
  //   if (ir_low_since == 0) ir_low_since = now;
  //   if ((now - ir_low_since) >= IR_DEBOUNCE_LOW_MS) ir_debounced_low = true;
  // } else {
  //   ir_low_since = 0;
  //   if (ir_high_since == 0) ir_high_since = now;
  //   if ((now - ir_high_since) >= IR_DEBOUNCE_HIGH_MS) ir_debounced_low = false;
  // }

  // ──────────── 3.  State-Übergänge ─────────────────────────
  switch (state) {

    case NO_BALL:
      // if (ir_debounced_low) {
      //   // Ball in Kuhle aber Kamera sieht nichts → direkt zum Tor
      //   state = DRIVE_TO_GOAL;
      //   pidCenterX.reset();
      //   pidCenterY.reset();
      // } 
       if (ballVisible) {
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

    // case DRIVE_TO_GOAL:
    //   if (!ir_debounced_low) {
    //     // Ball ist raus → Kamera übernimmt wieder
    //     state = CHASE_BALL;
    //     pidBallX.reset();
    //     pidBallY.reset();
    //    } 
      //  else if (p_y >= GOAL_TARGET_Y &&
      //            fabsf(p_y - GOAL_TARGET_Y) < GOAL_ARRIVED_DIST) {
      //   // Am Tor angekommen → Ball abgegeben
      //   state = NO_BALL;
      //   pidCenterX.reset();
      //   pidCenterY.reset();
      // }
      // break;
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

      Vec2 ball = {bx, by};

      Vec2 Player = {p_x, p_y};

      // Vec2 v = keeper(Player,ball);

      driveCmd.x = pidBallX.update(-v.x);
      driveCmd.y = pidBallY.update(-v.y);

      rotCmd = pidGyro.update(g_a);

      // ── Kick-Logik: Ball in Kuhle per Koordinaten ──────────
      bool ballInKuhle = (fabsf(bx) < KICK_BALL_THRESH)
                      && (fabsf(by) < KICK_BALL_THRESH);

      if (is_kicking) {
        // Gate nach 10 ms wieder schließen
        if ((now - kick_start_ms) >= KICK_GATE_MS) {
          digitalWrite(kickerPin, LOW);
          is_kicking = false;
        }
      } else if (ballInKuhle
              && (now - last_kick_time_ms) >= KICK_COOLDOWN_MS) {
        // Kick auslösen
        digitalWrite(kickerPin, HIGH);
        is_kicking      = true;
        kick_start_ms   = now;
        last_kick_time_ms = now;
      }

      break;
    }

    // case DRIVE_TO_GOAL: {


    //   // X → Feldmitte (zentriert aufs Tor zufahren)
    //   // Y → Richtung gegnerisches Tor
    //   float errX = 0.0f - p_x;
    //   float errY = GOAL_TARGET_Y - p_y;   

    //   driveCmd.x = pidCenterX.update(errX*1.5f);
    //   driveCmd.y = pidCenterY.update(-errY);

    //   // Zielwinkel in Grad (0° = +Y Richtung, positiv = rechts)
    //   float goalAngleDeg = atan2f(errX, errY) * (180.0f / PI);
    //   // Heading-Fehler: Differenz zwischen Soll und Ist
    //   float headingErr = goalAngleDeg - g_a;
 
    //    rotCmd = pidGyro.update(-headingErr);


    //   break;
    // }
  }

    applyFieldBounds(driveCmd, p_x, p_y, kBounds);

  // float transMag = sqrtf(driveCmd.x * driveCmd.x + driveCmd.y * driveCmd.y);
  // float scaledRot = rotAuth.apply(goalRotCmd, g_a, transMag); 


    if (state == DRIVE_TO_GOAL){

          Drive.calcDriveRotation(driveCmd.x, driveCmd.y, -rotCmd, -g_g);

    }
    else {
    Drive.calcDrive(driveCmd.x, driveCmd.y, -rotCmd);
    }
        Drive.drive();

    }


    else {
      
    int motorVRpin[3] = {3, 4, 15}; 
  int motorHRpin[3] = {5, 6, 19}; 
  int motorHLpin[3] = {10, 7, 14};
  int motorVLpin[3] = {12, 11, 18};
    // Drive.setMotor(motorVRpin[0], motorVRpin[1], motorVRpin[2], 255);
    // Drive.setMotor(motorHRpin[0], motorHRpin[1], motorHRpin[2], 255);
    // Drive.setMotor(motorHLpin[0], motorHLpin[1], motorHLpin[2], 255);
    // Drive.setMotor(motorVLpin[0], motorVLpin[1], motorVLpin[2], 255);


    for (int i = 0; i < 3; i++) {
         digitalWrite(motorVRpin[i], LOW);
         digitalWrite(motorHRpin[i], LOW);
         digitalWrite(motorHLpin[i], LOW);
         digitalWrite(motorVLpin[i], LOW);
    }

 
  }

    // Drive.calcDriveRotation(driveCmd.x, driveCmd.y, -scaledRot,);
  




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


