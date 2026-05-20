#ifndef MAIN_H
#define MAIN_H

#include "StepperLib.hpp"
#include <FastLED.h>

#define LEFT_DRIVE_MOTOR_ID 0x01 
#define LEFT_DRIVE_MOTOR_TYPE PWM
#define LEFT_DRIVE_MOTOR_DIR_PIN 0x05

#define RIGHT_DRIVE_MOTOR_ID 0x02
#define RIGHT_DRIVE_MOTOR_TYPE PWM
#define RIGHT_DRIVE_MOTOR_DIR_PIN 0x06

#define EXC_MOTOR_ID 0x04
#define EXC_MOTOR_TYPE ISV2

#define DEP_MOTOR_ID 0x00
#define DEP_MOTOR_TYPE PWM
#define DEP_MOTOR_DIR_PIN 0x04

#define NUM_LEDS 203
#endif