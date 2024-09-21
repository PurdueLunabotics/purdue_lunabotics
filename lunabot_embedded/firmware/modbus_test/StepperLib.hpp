#ifndef STEPPERLIB_H
#define STEPPERLIB_H
#include <Arduino.h>

#define OPERATING_MODE_SINGLE_DATA 0x06
#define RS485_TX_CONTROL 23 // RS485 Direction control
#define RS485Serial Serial2 // TODO - set these on the actual system
#define RS485_BAUD 19200

#define RS485Transmit HIGH
#define RS485Receive LOW

#define USE_DEFAULT 0

class StepperMotor {
public:
  uint16_t def_speed;
  uint16_t def_acceleration;
  uint16_t def_deceleration;

  StepperMotor(uint8_t MotorID, uint16_t def_speed = 500, uint16_t def_acceleration = 250, uint16_t def_deceleration = 250);

  void write_estop();
  void move_at_speed(uint16_t speed, uint16_t acceleration = USE_DEFAULT, uint16_t deceleration = USE_DEFAULT);
  void move_to_abs_pos(uint32_t position, uint16_t speed = USE_DEFAULT, uint16_t acceleration = USE_DEFAULT, uint16_t deceleration = USE_DEFAULT);
  void move_to_rel_pos(uint32_t position, uint16_t speed = USE_DEFAULT, uint16_t acceleration = USE_DEFAULT, uint16_t deceleration = USE_DEFAULT);

private:
  uint8_t MotorID;
  void write_short_frame(uint16_t param, uint16_t data);
  void write_multi_byte(uint16_t param, uint32_t data);
  void trigger_motion();
};
#endif