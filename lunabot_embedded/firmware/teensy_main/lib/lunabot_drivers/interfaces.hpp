#ifndef __INTERFACES_H__
#define __INTERFACES_H__

#include <ADS1119.h>

#include "robot.hpp"
#include <Arduino.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <Wire.h>
#include <Encoder.h>
#include <RobotMsgs.pb.h>
#include <StepperLib.hpp>
#include <FastLED.h>
#include "main.hpp"

enum MotorDir { CW = HIGH,
                CCW = LOW };
enum STMotor { M1 = 1,
               M2 = 2 };

class Sabertooth_MotorCtrl {
public:
  Sabertooth_MotorCtrl(Sabertooth *st, STMotor motor);
  void write(int8_t pwm);
  static void init_serial(HardwareSerial &s, int baud_rate);

private:
  static int initialized_serial_;
  Sabertooth *st_;
  STMotor motor_;
};

// Sensors
class ADS1119_Current_Bus {
public:
  ADS1119_Current_Bus() {};
  static void init_ads1119();
  static float read(uint8_t mux);

  static float adc_to_current_31A(float adc_value, float adc_fsr = 4.096, float vcc = 3.3);

private:
  static constexpr uint8_t ads1_addr = 0x41;

  static ADS1119Configuration configuration;
  static ADS1119 ads1;
};

class Led_Strip {
public:
  Led_Strip() {};
  static void init();
  static void set_color(int32_t, uint8_t);
private:
  static constexpr int BRIGHTNESS = 255; // 0 -> 255
  static CRGB all_led[NUM_LEDS];
};

class Encoder_Bus {
public:
  Encoder_Bus() {};
  static void init(uint8_t is_top);
  static long read(uint8_t id);

private:
  static constexpr int NUM_ACTUATORS = 2;
  static constexpr int PIN_LIST[NUM_ACTUATORS * 2] = {6, 7, 8, 9};

  static Encoder encs[NUM_ACTUATORS];
};

#endif
