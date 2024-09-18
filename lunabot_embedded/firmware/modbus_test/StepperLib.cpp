#include <ArduinoRS485.h>
#include "StepperLib.h"
#include <Arduino.h>

#define OPERATING_MODE_SINGLE_DATA 0x06

struct Addrs {
  uint16_t ControlReg = 0x6002;
} Addrs;

// compute CRC across bits 0 to data_range, store in data_range + 1 and data_range + 2
void modbusCRC(uint8_t *buf, int data_range) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < data_range; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  buf[data_range] = (uint8_t)(crc & 0xFF);
  buf[data_range + 1] = (uint8_t)(crc >> 8);
}

void setup_motors() {
  RS485.begin(9600);
}

void write_short_frame(uint8_t motor_id, uint16_t param, uint16_t data) {
  uint8_t buf[8] = {motor_id, OPERATING_MODE_SINGLE_DATA, (uint8_t)(param >> 8), (uint8_t)(param & 0xFF),
                    (uint8_t)(data >> 8), (uint8_t)(data & 0xFF), 0, 0};
  modbusCRC(buf, 6);
  RS485.beginTransmission();
  RS485.write(buf, 8);

  Serial.println("Writing data: ");
  for (int j = 0; j < 8; j++) {
    Serial.print(buf[j]);
  }

  RS485.endTransmission();
}

void write_estop(uint8_t motor_id) {
  write_short_frame(motor_id, Addrs.ControlReg, 0x40);
}
