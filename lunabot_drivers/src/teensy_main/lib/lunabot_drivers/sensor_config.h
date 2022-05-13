#include <Arduino.h>
#include <Stepper.h>
#include "actuator_config.h"

#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__

#define EXC_UPDATE_PERIOD 10000000
#define EXC_LOAD_DATA_PIN 10
#define EXC_LOAD_CLK_PIN 12

#define DEP_HALL_PIN 32
#define LIN_ACT_HALL_PIN 33
#define LEAD_SCREW_HALL_PIN 34

#define DT_FRONT_LEFT_ENC_A_PIN 10
#define DT_FRONT_LEFT_ENC_A_PIN 10
#define DT_FRONT_RIGHT_ENC_A_PIN 11
#define DT_FRONT_RIGHT_ENC_A_PIN 11
#define DT_BACK_LEFT_ENC_A_PIN 12
#define DT_BACK_LEFT_ENC_B_PIN 12
#define DT_BACK_RIGHT_ENC_A_PIN 13
#define DT_BACK_RIGHT_ENC_B_PIN 13

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

enum ExcState
{
    EMPTY,
    FILLING,
    FULL
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

struct ExcLoadCell
{
    float calibration_factor;
    float max_wt;
    float current_wt;
    uint8_t gain;
    ExcState state;

} exc_load;

struct EncoderConfig
{
    uint8_t A_pin;
    uint8_t B_pin;
};

const EncoderConfig front_left_enc = { .A_pin = 10, .B_pin = 11 };
const EncoderConfig front_right_enc = { .A_pin = 10, .B_pin = 11 };
const EncoderConfig back_left_enc = { .A_pin = 10, .B_pin = 11 };
const EncoderConfig back_right_enc = { .A_pin = 10, .B_pin = 11 };


void init_hall(uint8_t pin, void (*cb)());

#endif