#include <Arduino.h>
#include <Stepper.h>
#include "actuator_config.h"

#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__

#define INT(a) static_cast<int>(a)

#define EXC_UPDATE_PERIOD 10000000
#define EXC_LOAD_DATA_PIN 10
#define EXC_LOAD_CLK_PIN 12

#define EXC_CURRENT_PIN 10

#define DEP_HALL_PIN 32
#define LIN_ACT_HALL_PIN 33
#define LEAD_SCREW_HALL_PIN 34

#define DT_FT_LFT_ENC_A_PIN 10
#define DT_FT_LFT_ENC_B_PIN 10
#define DT_FT_RT_ENC_A_PIN 11
#define DT_FT_RT_ENC_B_PIN 11
#define DT_BK_LFT_ENC_A_PIN 12
#define DT_BK_LFT_ENC_B_PIN 12
#define DT_BK_RT_ENC_A_PIN 13
#define DT_BK_RT_ENC_B_PIN 13

/*
Using hall sensors to define joint limit logic for actuators

* Assuming starting from the robot's stored config

Actuation:
    Lead screw:
        State machine:  STORED -> FULL_EXT -> STORED
    Angle:
        State machine:  STORED -> START_EXC -> FULL_EXT -> STOP_EXC -> STORED

Deposition:
    State machine: STORED -> FULL_EXT -> STORED
    State machine: EMPTY -> FILLING -> FULL
*/

enum class DepState
{
    STORED,
    FULL_EXT,
    CNT
};

enum class BinState
{
    EMPTY,
    FILLING,
    FULL,
};

enum class ExcState
{
    NOMINAL,
    OVERCURRENT,
};

enum class LinActState
{
    STORED,    // move lin act
    START_EXC, // start excavating slowly
    FULL_EXT,  // excavate max speed
    STOP_EXC,  // stop lin act
    CNT
};

enum class LeadScrewState
{
    STORED,
    FULL_EXT,
    CNT
};

enum Limit
{
    AT_LIMIT,
    FREE
};

struct HallSensor
{
    uint8_t state;
    Limit lim;

}; 

struct ExcFeedback
{
    float calibration_factor;
    float max_wt;
    float current_wt;
    uint8_t gain;
    uint8_t bin_state;
    uint8_t exc_state;

};

struct CurrentSensor
{
    int current_value;
    int max_value;

};

void init_hall(uint8_t pin, void (*cb)());

#endif