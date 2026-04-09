#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "interfaces.hpp"
#include <Arduino.h>


#define LEFT_DRIVE_MOTOR_ID 0x01 // TODO RJN - set these ids on robot
#define RIGHT_DRIVE_MOTOR_ID 0x02
#define DEP_MOTOR_ID 0x03

namespace drivetrain {
void begin();
void cb(int32_t, int32_t, bool);
void update(float &, float &, float &, float &, float &, float &);
float update_curr_left();
float update_curr_right();
} // namespace drivetrain

namespace LEDs {
void cb(int32_t);
}

namespace deposition {
void begin();
void cb(int32_t, bool);
void update(float &);
float update_curr();
} // namespace deposition

#endif
