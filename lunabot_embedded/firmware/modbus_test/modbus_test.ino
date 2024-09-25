#include "StepperLib.hpp"
#include "Arduino.h"

#define MyMotorID 0x1

StepperMotor MyMotor(MyMotorID);

void setup(void) {
  Serial.begin(115200);
  Serial.println("Serial connected");

  // delay(1000);
  // MyMotor.write_estop();

  // Serial.println("Sample 1:");
  // MyMotor.move_to_abs_pos(200000);
  // delay(1000);
  // MyMotor.write_estop();

  // Serial.println("Sample 2:");
  // MyMotor.move_to_rel_pos(10000);
  // delay(1000);
  // MyMotor.write_estop();

  MyMotor.move_at_speed(600);
  delay(1000);
  // MyMotor.write_estop();

  // Serial.println("Sample 4 (ish):");
  // MyMotor.move_to_abs_pos(-200000); // feeding a negative number is fine
  // delay(1000);
  // MyMotor.write_estop();

  // Serial.println("Sample 1 (no defaults):");
  // MyMotor.move_to_abs_pos(200000, 600, 50, 50);
  // delay(1000);
  // MyMotor.write_estop();
}

void loop(void) {
  Serial.println();

  MyMotor.print_motor_state();

  Serial.println("Motor Position Raw: ");
  Serial.println(MyMotor.read_motor_position_raw());

  Serial.println("Motor Position Radians: ");
  Serial.println(MyMotor.read_motor_position_radians());

  Serial.println("Velocity (RPM): ");
  Serial.println(MyMotor.read_velocity());

  Serial.println("Velocity (RPM - raw): ");
  Serial.println(MyMotor.read_raw_velocity());

  Serial.println("Torque (\% of rated)");
  Serial.println(MyMotor.read_torque());

  Serial.println("Current (A): ");
  Serial.println(MyMotor.read_current());

  Serial.println("Voltage (V): ");
  Serial.println(MyMotor.read_voltage());

  Serial.println("Temperature (deg C): ");
  Serial.println(MyMotor.read_temperature());

  Serial.println("Over Load Ratio (%): ");
  Serial.println(MyMotor.read_over_load_ratio());

  Serial.println("Regen Load Ration (%): ");
  Serial.println(MyMotor.read_regen_load_ratio());
  delay(500);
}
