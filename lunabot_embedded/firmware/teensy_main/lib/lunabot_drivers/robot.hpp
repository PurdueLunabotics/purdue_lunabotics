#include "interfaces.hpp"
#include <Arduino.h>
#include <Stepper.h>

#ifndef __ROBOT_H__
#define __ROBOT_H__

#define ST_SERIAL Serial1
#define ST_BAUD_RATE 9600

extern ADS1115_lite adc0;
extern ADS1115_lite adc1;

// MCs
extern Sabertooth MC1; // top
extern Sabertooth MC2; // middle
extern Sabertooth MC3; // bottom
extern Sabertooth MC4; // high-current

namespace actuation {
void cb(int8_t lead_screw, int8_t lin_act);
void loop_once();
void update(int32_t *, int32_t *);
} // namespace actuation

namespace drivetrain {
void cb(int8_t left, int8_t right);
void loop_once();
void update(int32_t *, int32_t *);
} // namespace drivetrain

namespace deposition {
void cb(int8_t dep);
void loop_once();
void update(int32_t *);
} // namespace deposition

namespace excavation {
void cb(int8_t exc);
void loop_once();
void update(int32_t *);
} // namespace excavation

#endif
