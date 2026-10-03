#ifndef MAIN_H
#define MAIN_H

#define LEFT_DRIVE_MOTOR_ID 0x01 
#define LEFT_DRIVE_MOTOR_TYPE BLD305S
#define RIGHT_DRIVE_MOTOR_ID 0x02
#define RIGHT_DRIVE_MOTOR_TYPE BLD305S
#define EXC_MOTOR_ID 0x04
#define EXC_MOTOR_TYPE BLD305S
#define DEP_MOTOR_ID 0x03
#define DEP_MOTOR_TYPE BLD305S

class Led_Strip {
private:
  static constexpr int NUM_LEDS = 90;
  static constexpr int BRIGHTNESS = 255; // 0 -> 255
  static CRGB all_led[NUM_LEDS];
};
#endif