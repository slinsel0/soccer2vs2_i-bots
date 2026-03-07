/*
  LD19.cpp - Optimized for Teensy 4.0 / RoboCup
*/

#include "Arduino.h"
#include "LD19.h"
#include "GyroSystem.h"

extern GyroSystem gyro;

LD19::LD19()
{
}

void LD19::begin(HardwareSerial *_port)
{
  begin(_port, false);
}

void LD19::begin(HardwareSerial *_port, bool _debug)
{
  serialPort = _port;
  // LD19 Standard Baudrate ist 230400
  serialPort->begin(230400, SERIAL_8N1);
  // Note: addMemoryForRead removed from library. 
  // Add 'Serial1.addMemoryForRead(buf, 1024)' in main.cpp setup() if needed.
  debug = _debug;
}

void LD19::loop()
{
  // Lese so viele Bytes wie verfügbar sind
  while (serialPort->available() > 0)
  {
    uint8_t byte = serialPort->read();
    
    static int state = 0;
    static int idx = 0;
    
    if (state == 0) {
        if (byte == startchar) {
            state = 1;
            idx = 0;
        }
    } else if (state == 1) {
        buf[idx++] = byte;
        if (idx >= BUFFER_SIZE) {
            // Paket komplett
            processPacket();
            state = 0;
        }
    }
  }
}

// Hilfsfunktion zum Verarbeiten eines kompletten Pakets
#include "Arduino.h"
#include "LD19.h"
#include "GyroSystem.h" // <-- NEU: Gyro einbinden

extern GyroSystem gyro; // <-- NEU: Globale Gyro-Instanz holen

// ... [begin() und loop() bleiben unverändert] ...

// Hilfsfunktion zum Verarbeiten eines kompletten Pakets
void LD19::processPacket() 
{
    // 1. CRC Check
    uint8_t crcbuffer[46];
    crcbuffer[0] = 0x54; // Header wiederherstellen für CRC
    for (int i = 0; i < 45; i++) crcbuffer[i + 1] = buf[i];
    
    uint8_t crcc = CalCRC8(crcbuffer, 46);
    if (crcc != buf[45]) return; // CRC Fail -> Drop

    // 2. Parsen
    uint16_t startAngle = AddLSB(buf[startAngleLSB], buf[startAngleMSB]);
    uint16_t endAngle   = AddLSB(buf[endAngleLSB], buf[endAngleMSB]);

    // Winkel-Differenz korrigieren
    uint16_t angleDiff = (endAngle >= startAngle) ? (endAngle - startAngle) : ((endAngle + 36000) - startAngle);
    
    // NEU: Exakten Gyro-Winkel JETZT für diese 12 Punkte holen
    float current_robot_angle = gyro.getAngleRadians(); 
    
    for (int i = 0; i < 12; i++)
    {
        uint16_t dist = AddLSB(buf[5 + 3 * i], buf[6 + 3 * i]);
        uint8_t  intens = buf[7 + 3 * i];

        uint16_t angle_stepped = startAngle + (angleDiff * i) / 12;
        if (angle_stepped >= 36000) angle_stepped -= 36000;

        current_point_cloud_position++;
        if (current_point_cloud_position >= POINTCLOUD_SIZE) current_point_cloud_position = 0;

        // Umrechnung in kartesische Koordinaten für LidarProcessing
        float rad = (float)angle_stepped * (PI / 18000.0f);
        
        lidar_points[current_point_cloud_position].x = sinf(rad) * dist;
        lidar_points[current_point_cloud_position].y = cosf(rad) * dist;
        lidar_points[current_point_cloud_position].intensity = intens;
        
        // NEU: Winkel abspeichern!
        lidar_points[current_point_cloud_position].robot_angle_rad = current_robot_angle; 
    }
}

uint16_t LD19::AddLSB(uint8_t Low, uint8_t High)
{
  return ((uint16_t)High << 8) | Low;
}

uint8_t LD19::CalCRC8(uint8_t *p, uint8_t len)
{
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}