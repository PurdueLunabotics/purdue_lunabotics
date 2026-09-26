#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "interfaces.hpp"
#include <Arduino.h>
#include <Sabertooth.h>

#define ST_SERIAL Serial4
#define ST_BAUD_RATE 9600

// MCs
extern Sabertooth MC1; // top
extern Sabertooth MC2; // middle
extern Sabertooth MC3; // bottom

namespace actuation {
void cb(int8_t, uint8_t, uint8_t);
void update(float &, int32_t &, int32_t &);
} // namespace actuation

namespace drivetrain {
void begin();
void cb(int32_t, int32_t, bool);
void update(float &, float &, float &, float &, float &, float &);
float update_curr_left();
float update_curr_right();
} // namespace drivetrain

namespace LEDs {
void cb(int32_t, uint8_t);
}

namespace deposition {
void begin();
void cb(int32_t, int32_t, bool);
void update(float &);
float update_curr();
} // namespace deposition

namespace excavation {
void begin();
void cb(int32_t, bool);
void update(float &, float &, float &);
float update_curr();
} // namespace excavation
#endif
