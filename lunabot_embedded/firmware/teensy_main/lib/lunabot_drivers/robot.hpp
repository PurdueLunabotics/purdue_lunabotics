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
void update(int32_t &, int32_t &, float &, float &);

} // namespace actuation

namespace drivetrain {
void cb(int8_t left, int8_t right);
void update(int32_t &, int32_t &, float &, float &);
} // namespace drivetrain

namespace uwb {
void update(float &d0, float &d1, float &d2);
} // namespace uwb

namespace deposition {
void cb(int8_t dep);
void update(int32_t &, float &);
} // namespace deposition

namespace excavation {
void cb(int8_t exc);
void update(int32_t &);
} // namespace excavation

namespace load_cell {
void update(float &d1, float &d2);
}
#endif
