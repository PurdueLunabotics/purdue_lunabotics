#include <Arduino.h>
#include <Stepper.h>

#ifndef __ACTUATOR_CONFIG_H__
#define __ACTUATOR_CONFIG_H__

#define DRIVE_MTR_CNT 4
#define ACT_MTR_CNT 2
#define DEP_MTR_CNT 1
#define EXC_MTR_CNT 1

// ACTUATORS

enum StepperDir
{
    RETRACT = -1,
    EXTEND = 1
};
struct StepperConfig
{
    uint8_t PWM1_pin;
    uint8_t PWM2_pin;
    uint8_t DIR1_pin;
    uint8_t DIR2_pin;
    int steps;     // steps per revolution
    int speed;     // steps per minute
    int step_size; // step cnt per step
};

enum MotorDir
{
    CW = HIGH,
    CCW = LOW
};
struct MotorConfig
{
    uint8_t DIR_pin;
    uint8_t PWM_pin;
    uint8_t MAX_PWM = 255;
};

static const struct ExcavationConfig
{
    MotorConfig exc = {.DIR_pin = 26, .PWM_pin = 24};
} excavation_cfg;

static const struct DrivetrainConfig
{
    MotorConfig left_front = {.DIR_pin = 16, .PWM_pin = 15};
    MotorConfig right_front = {.DIR_pin = 14, .PWM_pin = 13};
    MotorConfig left_back = {.DIR_pin = 20, .PWM_pin = 19};
    MotorConfig right_back = {.DIR_pin = 23, .PWM_pin = 22};
} drivetrain_cfg;

static const struct DepositionConfig
{
    MotorConfig dep_motor = {.DIR_pin = 5, .PWM_pin = 4};
} deposition_cfg;

static const struct ActuationConfig
{
    MotorConfig left_lin_act = {.DIR_pin = 29, .PWM_pin = 28};
    MotorConfig right_lin_act = {.DIR_pin = 27, .PWM_pin = 25};
    StepperConfig lead_screw = {
        .PWM1_pin = 2, .PWM2_pin = 0, .DIR1_pin = 3, .DIR2_pin = 1, .steps = 200, .speed = 90, .step_size = 30};
} actuation_cfg;

void init_stepper(StepperConfig cfg, Stepper *stepper);
void stepper_on(StepperConfig s);
void stepper_off(StepperConfig s);
void stepper_step(StepperConfig s, Stepper *stepper, StepperDir dir);

void init_motor(MotorConfig m);
void write_motor(MotorConfig m, uint8_t pwm, MotorDir dir);
void stop_motor(MotorConfig m);

#endif