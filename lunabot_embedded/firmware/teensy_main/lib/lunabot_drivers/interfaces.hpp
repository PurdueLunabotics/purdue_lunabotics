#ifndef __INTERFACES_H__
#define __INTERFACES_H__

#include <ADS1115_lite.h>
#include <Arduino.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <Wire.h>
#include <Encoder.h>
#include <HX711.h>
#include <RobotMsgs.pb.h>
#include "robot.hpp"

#define UWBSerial Serial8

enum MotorDir { CW = HIGH, CCW = LOW };
enum STMotor { M1 = 1, M2 = 2 };

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

  static float adc_to_current_31A(float adc_value, float adc_fsr = 4.096, float vcc = 3.3);
  static float adc_to_current_15A(float adc_value, float adc_fsr = 4.096, float vcc = 3.3);

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

class KillSwitchRelay {
public:
  static bool dead;
  static long kill_time;

  static void init();
  
  //these two functions deal with the relay that kills all power to the motors
  static void reset();
  static void kill();

  //the main loop, to be run before the effort values are assigned to motors
  static void logic(RobotEffort &effort);

  //sets an individual motor to 0% power.
  static void kill_motor(int id, RobotEffort &effort);

private:
  //the pin to cut power to all motors. Active low to kill
  static constexpr int kill_pin = 11;
  static constexpr float drive_kill_curr = 7.0;
  static constexpr float exdep_kill_curr = 25.0;

  //the threshold at which the motor is set to 0% power 
  static constexpr int cutoff_thresh = 1000;
  //the threshold at which a motor set a 0% power is allowed to turn back on
  static constexpr int reset_thresh = 500;

  //Every cycle that a motor is overcurrent, a counter increases by this amount
  static constexpr int cutoff_increase = 3;
  //Every cycle, that counter decreases by this amount
  static constexpr int cutoff_decay = 1;

  //The relay that kills all motors must be dead for at least this long before resetting
  static constexpr int relay_dead_time = 2000;

  //If a motor has been set to 0% this many times, activate the kill relay. 
  static constexpr int kill_thresh = 3;

  volatile static float cutoff_buffer[4];
  volatile static float kill_buffer[4];
  volatile static bool is_dead[4];
};

class AMT13_Angle_Bus {
public:
  AMT13_Angle_Bus(){};
  static float read_enc(uint8_t id);

private:
  static constexpr int NUM_ENCODERS = 3;
  static constexpr int PIN_LIST[NUM_ENCODERS*2] = {4, 5, 6, 7, 8, 9};
  static constexpr float pulses_per_rev = 800; //4 times the value set on the encoders
  static constexpr float deg_per_rev = 360; //TODO, remove the deg2rad conversion and just to rad here
   
  static Encoder encs[NUM_ENCODERS];
};

#endif
