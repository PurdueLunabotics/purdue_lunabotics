#include "config.h"

void init_motor(MotorConfig m)
{
    pinMode(m.DIR_P, OUTPUT);
    pinMode(m.PWM_P, OUTPUT);
}

void init_stepper(StepperConfig cfg, Stepper* stepper)
{
    pinMode(cfg.PWM1_P, OUTPUT);
    pinMode(cfg.PWM2_P, OUTPUT);
    stepper->setSpeed(cfg.speed);
}

void write_motor(MotorConfig m, uint8_t pwm, MotorDir dir)
{
    digitalWrite(m.DIR_P, dir);
    analogWrite(m.PWM_P, min(pwm, m.MAX_PWM));
}

void step(StepperConfig s, Stepper* stepper, StepperDir dir) {
    stepper->step(s.step_size * dir);
}

void stop_motor(MotorConfig m) {
    analogWrite(m.PWM_P,0);
}