#ifndef RADARSENSOR_H
#define RADARSENSOR_H

#include <Arduino.h>
#include <SoftwareSerial.h>

typedef struct RadarTarget {
  float distance;  // mm
  float angle;     // radians
  float speed;     // cm/s
  int16_t x;       // mm
  int16_t y;       // mm
  bool detected;
} RadarTarget;

class RadarSensor {
  public:
    RadarSensor(uint8_t rxPin, uint8_t txPin);
    void begin(unsigned long baud = 256000);
    bool update();
    RadarTarget getTarget(uint8_t i = 0); // with index
  private:
    SoftwareSerial radarSerial;
    RadarTarget targets[3];  // multpile targets
    bool parseData(const uint8_t *buffer, size_t len);
};

#endif