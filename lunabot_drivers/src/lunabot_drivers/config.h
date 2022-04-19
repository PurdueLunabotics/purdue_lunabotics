#include <Arduino.h>
#include <Stepper.h>

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define DRIVE_MTR_CNT 4
#define ACT_MTR_CNT 2
#define DEP_MTR_CNT 1
#define EXC_MTR_CNT 1


enum StepperDir { RETRACT = -1, EXTEND = 1 };
struct StepperConfig
{
    uint8_t PWM1_P;
    uint8_t PWM2_P;
    uint8_t DIR1_P;
    uint8_t DIR2_P;
    uint16_t steps;
    uint16_t speed;
    uint16_t step_size;
};

enum MotorDir { CW = HIGH, CCW = LOW };
struct MotorConfig
{
    uint8_t DIR_P;
    uint8_t PWM_P;
    uint8_t MAX_PWM = 255;
};

static const struct ExcavationConfig
{
    MotorConfig exc = {.DIR_P = 26, .PWM_P = 24};
} excavation_cfg;


static const struct DrivetrainConfig
{
    MotorConfig left_front = {.DIR_P = 14, .PWM_P = 13};
    MotorConfig right_front = {.DIR_P = 16, .PWM_P = 15};
    MotorConfig left_back = {.DIR_P = 20, .PWM_P = 19};
    MotorConfig right_back = {.DIR_P = 23, .PWM_P = 22};
} drivetrain_cfg;

static const struct DepositionConfig
{
    MotorConfig dep_motor = { .DIR_P = 5, .PWM_P = 4};
} deposition_cfg;

static const struct ActuationConfig
{
    MotorConfig left_lin_act = {.DIR_P = 29, .PWM_P = 28};
    MotorConfig right_lin_act = {.DIR_P = 27, .PWM_P = 25};
    StepperConfig lead_screw = {
        .PWM1_P = 2, .PWM2_P = 0, 
        .DIR1_P = 3, .DIR2_P = 1, 
        .steps = 200, .speed = 50, .step_size = 10
    };
} actuation_cfg;


void init_motor(MotorConfig m);
void init_stepper(StepperConfig cfg, Stepper* stepper);
void write_motor(MotorConfig m, uint8_t pwm, MotorDir dir);
void step(StepperConfig s, Stepper* stepper, StepperDir dir);
void stop_motor(MotorConfig m);

#endif