#include <ArduinoRS485.h>
#include "StepperLib.h"
#include <Arduino.h>

#define OPERATING_MODE_SINGLE_DATA 0x06

struct Addrs {
  uint16_t ControlReg = 0x6002;

  uint16_t PathMode = 0x6200;
  uint16_t Pos_H = 0x6201;
  uint16_t Pos_L = 0x6202;
  uint16_t Speed = 0x6203;
  uint16_t Acceleration = 0x6204;
  uint16_t Deceleration = 0x6205;

} Addrs;

struct ControlReg {
  uint8_t TrigPath = 0x10;
  uint8_t ESTOP = 0x40;
} ControlReg;

struct PathMode {
  uint16_t AbsolutePosMode = 0x0001;
  uint16_t RelativePosMode = 0x0041;
  uint16_t VelocityMode = 0x0002;
} PathMode;

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

void setup_motors(unsigned long baudrate) {
  RS485.begin(baudrate);
}

void write_short_frame(uint8_t motor_id, uint16_t param, uint16_t data) {
  uint8_t buf[8] = {motor_id, OPERATING_MODE_SINGLE_DATA, (uint8_t)(param >> 8), (uint8_t)(param & 0xFF),
                    (uint8_t)(data >> 8), (uint8_t)(data & 0xFF), 0, 0};
  modbusCRC(buf, 6);
  RS485.beginTransmission();
  RS485.write(buf, 8);

  // for (int j = 0; j < 8; j++) {
  //   Serial.print(buf[j], HEX);
  //   Serial.print(" ");
  // }

  RS485.endTransmission();
}

void write_estop(uint8_t motor_id) {
  write_short_frame(motor_id, Addrs.ControlReg, ControlReg.ESTOP);
}

void trigger_motion(uint8_t motor_id) {
  write_short_frame(motor_id, Addrs.ControlReg, ControlReg.TrigPath);
}

void write_path_position(uint8_t motor_id, uint32_t position) {
  uint16_t data_high = (uint16_t)(position >> 16);
  uint16_t data_low = (uint16_t)(position & 0xFFFF);
  write_short_frame(motor_id, Addrs.Pos_H, data_high);
  write_short_frame(motor_id, Addrs.Pos_L, data_low);
}

void move_at_speed(uint8_t motor_id, uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  write_short_frame(motor_id, Addrs.PathMode, PathMode.VelocityMode);
  write_short_frame(motor_id, Addrs.Speed, speed);
  write_short_frame(motor_id, Addrs.Acceleration, acceleration);
  write_short_frame(motor_id, Addrs.Deceleration, deceleration);
  trigger_motion(motor_id);
}

void move_to_abs_pos(uint8_t motor_id, uint32_t position, uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  write_short_frame(motor_id, Addrs.PathMode, PathMode.AbsolutePosMode);
  write_path_position(motor_id, position);
  write_short_frame(motor_id, Addrs.Speed, speed);
  write_short_frame(motor_id, Addrs.Acceleration, acceleration);
  write_short_frame(motor_id, Addrs.Deceleration, deceleration);
  trigger_motion(motor_id);
}

void move_to_rel_pos(uint8_t motor_id, uint32_t position, uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  write_short_frame(motor_id, Addrs.PathMode, PathMode.RelativePosMode);
  write_path_position(motor_id, position);
  write_short_frame(motor_id, Addrs.Speed, speed);
  write_short_frame(motor_id, Addrs.Acceleration, acceleration);
  write_short_frame(motor_id, Addrs.Deceleration, deceleration);
  trigger_motion(motor_id);
}