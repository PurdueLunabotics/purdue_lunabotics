#include "StepperLib.hpp"
#include "Arduino.h"

#define MyMotorID 1

void setup(void) {
  Serial.begin(115200);
  Serial.println("Serial connected");

  StepperMotor MyMotor(MyMotorID);

  Serial.println("Sample 1:");
  MyMotor.move_to_abs_pos(200000);
  delay(1000);
  MyMotor.write_estop();

  Serial.println("Sample 2:");
  MyMotor.move_to_rel_pos(10000);
  delay(1000);
  MyMotor.write_estop();

  Serial.println("Sample 3:");
  MyMotor.move_at_speed(600);
  delay(1000);
  MyMotor.write_estop();

  Serial.println("Sample 4 (ish):");
  MyMotor.move_to_abs_pos(-200000); // feeding a negative number is fine
  delay(1000);
  MyMotor.write_estop();

  Serial.println("Sample 1 (no defaults):");
  MyMotor.move_to_abs_pos(200000, 600, 50, 50);
  delay(1000);
  MyMotor.write_estop();
}

void loop(void) {
}
