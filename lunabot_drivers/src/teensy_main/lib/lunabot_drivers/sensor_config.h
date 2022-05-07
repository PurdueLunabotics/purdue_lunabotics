#include <Arduino.h>
#include <Stepper.h>

#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__

/*
Using hall sensors to define joint limit logic for actuators

* Assuming starting from the robot's stored config

Actuation:
    Lead screw: 2
        State machine:  STORED -> FULL_EXT -> STORED 
    Angle: (2 x 2) 
        State machine:  STORED -> START_EXC -> FULL_EXT 

Deposition:
    State machine: STORED -> FULL_EXT -> STORED 
    State machine: EMPTY -> FILLING -> FULL 
*/

static const struct DepHallSensor {
    enum FSM = { STORED, FULL_EXT, CNT };
    uint8_t data_P = 11; 
} dep_hall_sensor;

static const struct DepWeightSensor {
    enum FSM = { EMPTY, FILLING, FULL, CNT };
    uint8_t data_P;
    uint8_t clock_P;
    float calibration_factor;
} dep_weight_sensor;

static const struct LinActRightHallSensor {
    enum FSM = { STORED, START_EXC, FULL_EXT, CNT };
    uint8_t data_P; 
} left_lin_act_hall;

static const struct LinActLeftHallSensor {
    enum FSM = { STORED, START_EXC, FULL_EXT, CNT };
    uint8_t data_P; 
} right_lin_act_hall;

static const struct LeadScrewHallSensor {
    enum FSM = { STORED, FULL_EXT, CNT };
    uint8_t data_P; 
} lead_screw_hall;

void init_sensor();

#endif