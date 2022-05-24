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
    int steps; // steps per revolution
    int speed; // steps per minute
    int step_size; // step cnt per step
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
    MotorConfig left = {.DIR_P = 1, .PWM_P = 0, .MAX_PWM = 100};
    MotorConfig right = {.DIR_P = 3, .PWM_P = 2. MAX_PWM = 100};
} drivetrain_cfg;

static const struct DepositionConfig
{
    MotorConfig dep_motor = { .DIR_P = 16, .PWM_P = 15};
} deposition_cfg;

static const struct ActuationConfig
{
    MotorConfig lin_act = {.DIR_P = 5, .PWM_P = 4};
    StepperConfig lead_screw = {
        .PWM1_P = 22, .PWM2_P = 19, 
        .DIR1_P = 23, .DIR2_P = 20, 
        .steps = 200, .speed = 110, .step_size = 30
    };
} actuation_cfg;


void init_stepper(StepperConfig cfg, Stepper* stepper);
void stepper_on(StepperConfig s);
void stepper_off(StepperConfig s);
void stepper_step(StepperConfig s, Stepper* stepper, StepperDir dir);

void init_motor(MotorConfig m);
void write_motor(MotorConfig m, uint8_t pwm, MotorDir dir);
void stop_motor(MotorConfig m);

#endif