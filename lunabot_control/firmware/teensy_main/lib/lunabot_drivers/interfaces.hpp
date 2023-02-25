#include <Arduino.h>
#include <Sabertooth.h>
#include <Stepper.h>

#ifndef __CONFIG_H__
#define __CONFIG_H__

enum StepperDir { RETRACT = -1, EXTEND = 1 };

enum MotorDir { CW = HIGH, CCW = LOW };

enum STMotor { M1 = 1, M2 = 2 };

class StepperInterface {
  public:
    StepperInterface(uint8_t PWM1_P, uint8_t PWM2_P, uint8_t DIR1_P,
                     uint8_t DIR2_P, int steps, int speed, int step_size);
    void on();
    void off();
    void step(StepperDir dir);

  private:
    Stepper s_;
    uint8_t PWM1_P_;
    uint8_t PWM2_P_;
    uint8_t DIR1_P_;
    uint8_t DIR2_P_;
    int en_;
    int steps_;     // steps per revolution
    int speed_;     // steps per minute
    int step_size_; // step cnt per step
};

class MotorInterface {
  public:
    MotorInterface(uint8_t PWM_P, uint8_t DIR_P);
    void write(uint8_t pwm, MotorDir dir);

  private:
    uint8_t PWM_P_;
    uint8_t DIR_P_;
};

class STMotorInterface {
  public:
    STMotorInterface(Sabertooth *st, STMotor motor);
    void write(int8_t pwm);
    static void init_serial(HardwareSerial s, int baud_rate);

  private:
    static int initialized_serial_;
    Sabertooth *st_;
    STMotor motor_;
};

// static struct ExcavationConfig
// {
//     STMotorConfig exc = {.st = &MC2, .motor = STMotor::M1};
// } excavation_cfg;

// static struct DrivetrainConfig
// {
//     STMotorConfig left = {.st = &MC1, .motor = STMotor::M1};
//     STMotorConfig right = {.st = &MC1, .motor = STMotor::M2};
// } drivetrain_cfg;

// static const struct DepositionConfig
// {
//     MotorConfig dep_motor = {.PWM_P = 13, .DIR_P = 14};
// } deposition_cfg;

// static struct ActuationConfig
// {
//     STMotorConfig lin_act = {.st = &MC2, .motor = STMotor::M1};
//     StepperConfig lead_screw = {
//         .PWM1_P = 19, .PWM2_P = 22, .DIR1_P = 20, .DIR2_P = 23, .steps = 200,
//         .speed = 90, .step_size = 30};
// } actuation_cfg;

#endif