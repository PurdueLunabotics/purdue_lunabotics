#include <ADS1115_lite.h>
#include <Arduino.h>
#include <SPI.h>
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
    static void init_ads1115(ADS1115_lite *adc0, ADS1115_lite *adc1);
    int16_t read();

  private:
    static int initialized_;
    ADS1115_lite *adc_;
    ADSChannel ch_;
    int16_t curr_;
};

class EncoderBus {
  public:
    EncoderBus(){};
    static void init();
    static float read_enc(uint8_t id);
    static void transfer();

  private:
    static constexpr int BUFFER_SIZE = 2;
    static constexpr int BUS_SIZE = 5;
    static constexpr int sel0_p_ = 8;
    static constexpr int sel1_p_ = 9;
    static constexpr int sel2_p_ = 10;
    static constexpr int clk_p_ = 13;
    volatile static uint8_t curr_id_;
    volatile static uint8_t spi_buffer_[BUFFER_SIZE];
    volatile static uint16_t enc_buffer_[BUS_SIZE];

    static void select_enc_(uint8_t id);
};

#endif
