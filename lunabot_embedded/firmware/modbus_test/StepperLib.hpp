#ifndef STEPPERCLASS_H
#define STEPPERCLASS_H
#include <Arduino.h>

#define USE_DEFAULT 0

class StepperMotor {
public:
  uint16_t def_speed;
  uint16_t def_acceleration;
  uint16_t def_deceleration;

  StepperMotor(uint8_t MotorID, unsigned long baudrate, uint16_t def_speed = 600, uint16_t def_acceleration = 50, uint16_t def_deceleration = 50);

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