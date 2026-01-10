#ifndef LD19_h
#define LD19_h

#include "Arduino.h"
#include "CRCTable.h"

// Define globally so LidarProcessing can see it
#define POINTCLOUD_SIZE 600

class LD19
{
  private:
    static constexpr int BUFFER_SIZE = 47; // Safety Puffer
    
    // Protokoll Konstanten
    static constexpr int startchar = 0x54;
    static constexpr int startAngleLSB = 3;
    static constexpr int startAngleMSB = 4;
    static constexpr int endAngleLSB = 41;
    static constexpr int endAngleMSB = 42;

  public:
    LD19();
    void begin(HardwareSerial* _port);
    void begin(HardwareSerial* _port, bool _debug);
    void loop();

    struct lidarPointVec {
      float x; // Geändert auf float für Teensy FPU Performance
      float y;
      uint8_t intensity;
    };
    
    lidarPointVec lidar_points[POINTCLOUD_SIZE];

  private:
    HardwareSerial* serialPort;
    bool debug;
    uint8_t buf[BUFFER_SIZE];
    // Puffer entfernt, da addMemoryForRead im main setup gemacht werden sollte
    
    uint16_t current_point_cloud_position = 0;
    
    void processPacket();
    uint16_t AddLSB(uint8_t Low, uint8_t High);
    uint8_t CalCRC8(uint8_t *p, uint8_t len);
};

#endif