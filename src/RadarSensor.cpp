#include "RadarSensor.h"

RadarSensor::RadarSensor(uint8_t rxPin, uint8_t txPin)
  : radarSerial(rxPin, txPin)
{}

void RadarSensor::begin(unsigned long baud) {
  radarSerial.begin(baud);
}

// Parser state-machine for UART data
bool RadarSensor::update() {
  static uint8_t buffer[30]; 
  static size_t index = 0;
  static enum {WAIT_AA, WAIT_FF, WAIT_03, WAIT_00, RECEIVE_FRAME} state = WAIT_AA;

  bool data_updated = false;

  while (radarSerial.available()) {
    byte byteIn = radarSerial.read();

    switch(state) {
      case WAIT_AA:
        if(byteIn == 0xAA) state = WAIT_FF;
        break;

      case WAIT_FF:
        if(byteIn == 0xFF) state = WAIT_03;
        else state = WAIT_AA;
        break;

      case WAIT_03:
        if(byteIn == 0x03) state = WAIT_00;
        else state = WAIT_AA;
        break;

      case WAIT_00:
        if(byteIn == 0x00) {
          index = 0;
          state = RECEIVE_FRAME;
        } else state = WAIT_AA;
        break;

      case RECEIVE_FRAME:
        buffer[index++] = byteIn;
        if(index >= 26) { // 24 bytes data + 2 tail bytes
          if(buffer[24] == 0x55 && buffer[25] == 0xCC) {
            data_updated = parseData(buffer, 24);
          }
          state = WAIT_AA;
          index = 0;
        }
        break;
    }
  }
  return data_updated;
}

bool RadarSensor::parseData(const uint8_t *buf, size_t len) {
  if(len != 24)
    return false;

  for (int i = 0; i < 3; i++) {
    int offset = i * 8;
    int16_t raw_x = buf[offset + 0] | (buf[offset + 1] << 8);
    int16_t raw_y = buf[offset + 2] | (buf[offset + 3] << 8);
    int16_t raw_speed = buf[offset + 4] | (buf[offset + 5] << 8);
    uint16_t raw_pixel_dist = buf[offset + 6] | (buf[offset + 7] << 8);

    targets[i].detected = !(raw_x == 0 && raw_y == 0 && raw_speed == 0 && raw_pixel_dist == 0);

    // correctly parse signed values
    if (targets[i].detected) {
      targets[i].x = ((raw_x & 0x8000) ? 1 : -1) * (raw_x & 0x7FFF);
      targets[i].y = ((raw_y & 0x8000) ? 1 : -1) * (raw_y & 0x7FFF);
      targets[i].speed = ((raw_speed & 0x8000) ? 1 : -1) * (raw_speed & 0x7FFF);
      targets[i].distance = sqrt(targets[i].x * targets[i].x + targets[i].y * targets[i].y);
      
      // angle calculation (convert radians to degrees, then flip)
      float angleRad = atan2(targets[i].y, targets[i].x) - (PI / 2);
      float angleDeg = angleRad * (180.0 / PI);
      targets[i].angle = -angleDeg; // align angle with x measurement positive / negative sign
    } else {
      targets[i].x = 0.0;
      targets[i].y = 0.0;
      targets[i].speed = 0.0;
      targets[i].distance = 0.0;
      targets[i].angle = 0.0;
    }
  }
  
  return true;
}

RadarTarget RadarSensor::getTarget(uint8_t i) {
  if (i < 3) return targets[i];
  return targets[0];
}