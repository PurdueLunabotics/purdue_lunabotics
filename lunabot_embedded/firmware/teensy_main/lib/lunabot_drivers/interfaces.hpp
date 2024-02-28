#include <ADS1115_lite.h>
#include <Arduino.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <Stepper.h>
#include <Wire.h>
#include <HX711.h>

#ifndef __INTERFACES_H__
#define __INTERFACES_H__

#define UWBSerial Serial8

enum StepperDir { RETRACT = -1, EXTEND = 1 };
enum MotorDir { CW = HIGH, CCW = LOW };
enum STMotor { M1 = 1, M2 = 2 };

class Stepper2Phase_MotorCtrl {
public:
  Stepper2Phase_MotorCtrl(uint8_t PWM1_P, uint8_t PWM2_P, uint8_t DIR1_P, uint8_t DIR2_P, int steps,
                          int speed, int step_size);
  void on();
  void off();
  void step(StepperDir dir);

private:
  Stepper s_;
  uint8_t PWM1_P_;
  uint8_t PWM2_P_;
  uint8_t DIR1_P_;
  uint8_t DIR2_P_;
  int enabled_;
  int steps_;     // steps per revolution
  int speed_;     // steps per minute
  int step_size_; // step cnt per step
};

class PWM_MotorCtrl {
public:
  PWM_MotorCtrl(uint8_t PWM_P, uint8_t DIR_P);
  void write(uint8_t pwm, MotorDir dir);

private:
  uint8_t PWM_P_;
  uint8_t DIR_P_;
};

class Sabertooth_MotorCtrl {
public:
  Sabertooth_MotorCtrl(Sabertooth *st, STMotor motor);
  void write(int8_t pwm);
  static void init_serial(HardwareSerial s, int baud_rate);

private:
  static int initialized_serial_;
  Sabertooth *st_;
  STMotor motor_;
};

// Sensors

class ACS711_Current_Bus {
public:
  ACS711_Current_Bus(){};
  static void init_ads1115();
  static int16_t read(uint8_t bus, uint8_t mux);
  static void transfer();

private:
  static constexpr int BUSES = 2;
  static constexpr int CH_SIZE = 4;
  static constexpr int BUS_SIZE = 4;
  static const int ADS_CHANNELS[CH_SIZE];
  static int initialized_;
  static ADS1115_lite adc0_;
  static ADS1115_lite adc1_;
  static uint8_t adc0_ch_;
  static uint8_t adc1_ch_;
  static int16_t curr_buffer_[BUSES][BUS_SIZE];
};

class M5Stack_UWB_Trncvr {
public:
  M5Stack_UWB_Trncvr(){};
  static void init();
  static float read_uwb(uint8_t id);
  static void transfer();

private:
  static constexpr int NUM_UWB_TAGS = 3;
  volatile static float recv_buffer_[NUM_UWB_TAGS];
};

class VLH35_Angle_Bus {
public:
  VLH35_Angle_Bus(){};
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
  volatile static uint32_t enc_buffer_[BUS_SIZE];

  static void select_enc_(uint8_t id);
};

#endif
