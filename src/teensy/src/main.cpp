#include <Arduino.h>
#include "DriveSystem.h"
#include "LD19.h" 
#include "LidarProcessing.h"
#include <Wire.h>
#include "GyroSystem.h"
#include <PacketSerial.h>   // by Christopher Baker
#include <FastCRC.h>        // by Frank Boesing

COBSPacketSerial cobsSerial;
FastCRC32 CRC32;
GyroSystem gyro;

DriveSystem Drive;


float g_a =0;

// Frame-Struct exakt 22 Bytes, little-endian
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

  // (Optional) kleines Ack zurücksenden (gleiches Framing: COBS + CRC32)
  // Hier schicken wir msg_id=0xA1, seq zurück.
  uint8_t ack_body[2] = { 0xA1, pkt.seq };
  uint32_t ack_crc = CRC32.crc32(ack_body, sizeof(ack_body));
  uint8_t ack_frame[2 + 4];
  memcpy(&ack_frame[0], ack_body, 2);
  memcpy(&ack_frame[2], &ack_crc, 4);
  cobsSerial.send(ack_frame, sizeof(ack_frame)); // COBS + 0x00 übernimmt die Lib
}

void setup() {
      gyro.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(2000000);
  while (!Serial && millis() < 2000) { }
  cobsSerial.setStream(&Serial);
  cobsSerial.setPacketHandler(&onPacketReceived);
  Wire1.begin();
  Wire2.begin();
  Wire.begin();
    gyro.begin();
}

uint32_t lastBlink = 0;

void loop() {
  cobsSerial.update(); // wichtig: ruft intern den Decoder auf
  gyro.update();
  g_a = gyro.getAngleDegrees();
Drive.calcDrive(-last_vx,last_vy*2,g_a*0.5);   
Drive.drive();


  // Demo: LED-Frequenz zeigt Traffic
  if (got_cmd) {
    if (millis() - lastBlink > 50) {
      digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
      lastBlink = millis();
    }
    got_cmd = false;



  if (last_omega == 0){
    last_vy *= -last_vy;
  }
  else {
    last_vy = +last_vy;
  }
  




    // >>> Hier: Deine Regelung / Motorbefehle mit last_vx, last_vy, last_omega
    // z.B. setVelocity(last_vx, last_vy, last_omega);
  } else {
    if (millis() - lastBlink > 500) {
      digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
      lastBlink = millis();
    }
  }







}
