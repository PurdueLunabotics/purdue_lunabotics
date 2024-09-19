#include "StepperLib.hpp"
#include "ArduinoRS485.h"

#define OPERATING_MODE_SINGLE_DATA 0x06

bool serial_has_started = true;

struct Addrs {
  uint16_t ControlReg = 0x6002;

  uint16_t PathMode = 0x6200;
  uint16_t Pos = 0x6201; // high, low is +1
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

// Create a StepperMotor object
// MotorID and baudrate are required
// Optional default values for speed, acceleration, deceleration
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
StepperMotor::StepperMotor(uint8_t MotorID, unsigned long baudrate, uint16_t def_speed, uint16_t def_acceleration, uint16_t def_deceleration) {
  this->MotorID = MotorID;
  this->def_speed = def_speed;
  this->def_acceleration = def_acceleration;
  this->def_deceleration = def_deceleration;
  if (!serial_has_started) {
    serial_has_started = true;
    RS485.begin(baudrate);
  }
}

// emergency stops motor
void StepperMotor::write_estop() {
  write_short_frame(Addrs.ControlReg, ControlReg.ESTOP);
}

// move the motor at constant velocity, using default values for acceleration and deceleration unless specified
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void StepperMotor::move_at_speed(uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  if (acceleration == USE_DEFAULT) {
    acceleration = def_acceleration;
  }
  if (deceleration == USE_DEFAULT) {
    deceleration = def_deceleration;
  }

  write_short_frame(Addrs.PathMode, PathMode.VelocityMode);
  write_short_frame(Addrs.Speed, speed);
  write_short_frame(Addrs.Acceleration, acceleration);
  write_short_frame(Addrs.Deceleration, deceleration);
  trigger_motion();
}

// move the motor to an absolute position, using default values for speed, acceleration, and deceleration unless specified
// position in encoder pulses
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void StepperMotor::move_to_abs_pos(uint32_t position, uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  if (speed == USE_DEFAULT) {
    speed = def_speed;
  }
  if (acceleration == USE_DEFAULT) {
    acceleration = def_acceleration;
  }
  if (deceleration == USE_DEFAULT) {
    deceleration = def_deceleration;
  }

  write_short_frame(Addrs.PathMode, PathMode.AbsolutePosMode);
  write_multi_byte(Addrs.Pos, position);
  write_short_frame(Addrs.Speed, speed);
  write_short_frame(Addrs.Acceleration, acceleration);
  write_short_frame(Addrs.Deceleration, deceleration);
  trigger_motion();
}

// move the motor to a relative position, using default values for speed, acceleration, and deceleration unless specified
// position in encoder pulses
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void StepperMotor::move_to_rel_pos(uint32_t position, uint16_t speed, uint16_t acceleration, uint16_t deceleration) {
  if (speed == USE_DEFAULT) {
    speed = def_speed;
  }
  if (acceleration == USE_DEFAULT) {
    acceleration = def_acceleration;
  }
  if (deceleration == USE_DEFAULT) {
    deceleration = def_deceleration;
  }

  write_short_frame(Addrs.PathMode, PathMode.RelativePosMode);
  write_multi_byte(Addrs.Pos, position);
  write_short_frame(Addrs.Speed, speed);
  write_short_frame(Addrs.Acceleration, acceleration);
  write_short_frame(Addrs.Deceleration, deceleration);
  trigger_motion();
}

// internal function to send frame of modbus data on RS485
void StepperMotor::write_short_frame(uint16_t param, uint16_t data) {
  uint8_t buf[8] = {MotorID, OPERATING_MODE_SINGLE_DATA, (uint8_t)(param >> 8), (uint8_t)(param & 0xFF),
                    (uint8_t)(data >> 8), (uint8_t)(data & 0xFF), 0, 0};
  modbusCRC(buf, 6);
  RS485.beginTransmission();
  RS485.write(buf, 8);

  for (int j = 0; j < 8; j++) {
    Serial.print(buf[j], HEX);
    Serial.print(" ");
  }
  Serial.println();

  RS485.endTransmission();
}

// internal function to write 2 frames of data (32 bits)
void StepperMotor::write_multi_byte(uint16_t param, uint32_t data) {
  uint16_t data_high = (uint16_t)(data >> 16);
  uint16_t data_low = (uint16_t)(data & 0xFFFF);
  write_short_frame(param, data_high);
  write_short_frame(param + 1, data_low);
}

// internal function to start motion path (see motor docs for more info)
void StepperMotor::trigger_motion() {
  write_short_frame(Addrs.ControlReg, ControlReg.TrigPath);
}