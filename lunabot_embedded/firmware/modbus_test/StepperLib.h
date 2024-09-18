#ifndef STEPPPERLIB_H
#define SSTEPPPERLIB_H

// inits RS485 serial
void setup_motors(unsigned long baudrate);

// emergency stops motor
void write_estop(uint8_t motor_id);

// move the motor at constant velocity
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void move_at_speed(uint8_t motor_id, uint16_t speed, uint16_t acceleration = 50, uint16_t deceleration = 50);

// move the motor to an absolute position
// position in encoder pulses
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void move_to_abs_pos(uint8_t motor_id, uint32_t position, uint16_t speed = 600, uint16_t acceleration = 50, uint16_t deceleration = 50);

// move the motor to a relative posiion
// position in encoder pulses
// speed in rpm
// acceleration and deceleration in ms/1000 rpm
void move_to_rel_pos(uint8_t motor_id, uint32_t position, uint16_t speed = 600, uint16_t acceleration = 50, uint16_t deceleration = 50);
#endif