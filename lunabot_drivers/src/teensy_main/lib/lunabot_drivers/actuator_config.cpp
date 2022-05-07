#include "actuator_config.h"

void init_motor(MotorConfig m)
{
    pinMode(m.DIR_pin, OUTPUT);
    pinMode(m.PWM_pin, OUTPUT);
}

void init_stepper(StepperConfig cfg, Stepper *stepper)
{
    pinMode(cfg.PWM1_pin, OUTPUT);
    pinMode(cfg.PWM2_pin, OUTPUT);
    stepper->setSpeed(cfg.speed);
}

void write_motor(MotorConfig m, uint8_t pwm, MotorDir dir)
{
    digitalWrite(m.DIR_pin, dir);
    analogWrite(m.PWM_pin, pwm);
}

void stepper_step(StepperConfig s, Stepper *stepper, StepperDir dir)
{
    stepper->step(s.step_size * dir);
}

void stepper_on(StepperConfig s)
{
    digitalWrite(s.PWM1_pin, HIGH);
    digitalWrite(s.PWM2_pin, HIGH);
}

void stepper_off(StepperConfig s)
{
    digitalWrite(s.PWM1_pin, LOW);
    digitalWrite(s.PWM2_pin, LOW);
}

void stop_motor(MotorConfig m)
{
    analogWrite(m.PWM_pin, 0);
}