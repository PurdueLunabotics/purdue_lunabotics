#include "interfaces.hpp"
#include <Arduino.h>
#include <Stepper.h>

#ifndef __ROBOT_H__
#define __ROBOT_H__

#define ST_SERIAL Serial1
#define ST_BAUD_RATE 9600

namespace actuation {
void cb(int8_t lead_screw, int8_t lin_act);
void run();
} // namespace actuation

namespace drivetrain {
void cb(int8_t left, int8_t right);
}

namespace deposition {
void cb(int8_t dep);
}

namespace excavation {
void cb(int8_t exc);
}

#endif