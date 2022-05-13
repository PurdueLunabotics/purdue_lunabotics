// Quickstop.pde
// -*- mode: C++ -*-
//
// Check stop handling.
// Calls stop() while the stepper is travelling at full speed, causing
// the stepper to stop as quickly as possible, within the constraints of the
// current acceleration.
//
// Copyright (C) 2012 Mike McCauley
// $Id:  $
#include <Stepper.h>
// Define a stepper and the pins it will use

#define PWM1 2
#define PWM2 0
#define DIR1 3
#define DIR2 1

Stepper stepper(200, DIR1,DIR2); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
void setup()
{  
  stepper.setSpeed(50);
  //stepper.setAcceleration(200);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  digitalWrite(PWM1,HIGH);
  digitalWrite(PWM2,HIGH);
}
void loop()
{    
  stepper.step(-10);
}
