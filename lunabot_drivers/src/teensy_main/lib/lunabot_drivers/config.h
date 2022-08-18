#include <Arduino.h>
#include <Stepper.h>
#include <Sabertooth.h>

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define DRIVE_MTR_CNT 2
#define ACT_MTR_CNT 2
#define DEP_MTR_CNT 1
#define EXC_MTR_CNT 1

#define MC_SERIAL_BAUD_RATE 9600

extern Sabertooth MC1;
extern Sabertooth MC2;

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

struct SerialMotorConfig {

    Sabertooth* st;
    uint8_t motor;
    uint8_t MAX_SPEED = 127;
};

struct MotorConfig 
{
    uint8_t PWM_P;
    uint8_t DIR_P;
    uint8_t MAX_SPEED = 255;
};

static struct ExcavationConfig
{
    SerialMotorConfig exc = {  .st=&MC2, .motor = 2 };
} excavation_cfg;


static struct DrivetrainConfig
{
    SerialMotorConfig left = {.st=&MC1, .motor=1, .MAX_SPEED = 50 };
    SerialMotorConfig right = {.st=&MC1,  .motor=2, .MAX_SPEED = 50 };
} drivetrain_cfg;

static const struct DepositionConfig
{
    MotorConfig dep_motor = {  .PWM_P = 13, .DIR_P = 14 };
} deposition_cfg;

static struct ActuationConfig
{
    SerialMotorConfig lin_act = { .st=&MC2, .motor=1 };
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
void write_serial_motor(SerialMotorConfig m, int8_t power);
void stop_motor(MotorConfig m);

void init_serial();

#endif