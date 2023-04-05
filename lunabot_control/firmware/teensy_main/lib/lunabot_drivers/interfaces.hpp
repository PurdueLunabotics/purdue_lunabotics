#include <ADS1115_lite.h>
#include <Arduino.h>
#include <Sabertooth.h>
#include <Stepper.h>

#ifndef __INTERFACES_H__
#define __INTERFACES_H__

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

// Sensors

enum ADSChannel {
    A0_ch = ADS1115_REG_CONFIG_MUX_SINGLE_0,
    A1_ch = ADS1115_REG_CONFIG_MUX_SINGLE_1,
    A2_ch = ADS1115_REG_CONFIG_MUX_SINGLE_2,
    A3_ch = ADS1115_REG_CONFIG_MUX_SINGLE_3
};

class CurrentSensor {
  public:
    CurrentSensor(ADS1115_lite *adc, ADSChannel ch);
    static void init_ads1115(ADS1115_lite *adc);
    int16_t read();
    void loop();

  private:
    ADS1115_lite *adc_;
    ADSChannel ch_;
    int16_t curr_;
};

#endif
