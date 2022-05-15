#include <Arduino.h>
#include <Stepper.h>
#include "actuator_config.h"

#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__

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

enum DepState
{
    STORED,
    FULL_EXT,
    CNT
};

enum BinState
{
    EMPTY,
    FILLING,
    FULL,
};

enum ExcState
{
    NOMINAL,
    OVERCURRENT,
};

enum LinActState
{
    STORED,    // move lin act
    START_EXC, // start excavating slowly
    FULL_EXT,  // excavate max speed
    STOP_EXC,  // stop lin act
    CNT
};

enum LeadScrewState
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
    union
    {
        DepState dep_state;
        LinActState lin_act_state;
        LeadScrewState lead_screw_state;
    };
    Limit lim;

} dep_hall, lin_act_hall, lead_screw_hall;

struct ExcFeedback
{
    float calibration_factor;
    float max_wt;
    float current_wt;
    uint8_t gain;
    BinState bin_state;
    ExcState exc_state;

} exc_feedback;

struct CurrentSensor
{
    int current_value;
    int max_value;

} exc_current;

void init_hall(uint8_t pin, void (*cb)());

#endif