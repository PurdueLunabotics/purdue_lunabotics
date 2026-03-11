#include "StepperLib.hpp"
#include "Arduino.h"

<<<<<<< HEAD
StepperMotor MyMotor(0x1, BLD305S, 255, 255);
=======
#define MyMotorID 0x1

StepperMotor MyMotor_1(0x1);
StepperMotor MyMotor_2(0x2);
>>>>>>> develop

void setup(void) {
  Serial.begin(115200);
  Serial.println("Serial connected");

<<<<<<< HEAD
  MyMotor.begin();

  MyMotor.move_at_speed(200);
=======
  MyMotor_1.begin();
  MyMotor_2.begin();

  MyMotor_1.move_at_speed(600);
  MyMotor_2.move_at_speed(1200);
  delay(1000);
>>>>>>> develop
}

void loop(void) {
  // Serial.println();

  // MyMotor.print_motor_state();

  // Serial.println("Motor Position Raw: ");
  // Serial.println(MyMotor.read_motor_position_raw());

  // Serial.println("Motor Position Radians: ");
  // Serial.println(MyMotor.read_motor_position_radians());

<<<<<<< HEAD
  // Serial.println("1 Velocity (RPM): ");
  // Serial.println(MyMotor.read_velocity());
  Serial.println();

  if(MyMotor.read_velocity() < 250) {
    MyMotor.move_at_speed(1200);
  } else if (MyMotor.read_velocity() > 1000) {
    MyMotor.move_at_speed(100);
  }

=======
  Serial.println("1 Velocity (RPM): ");
  Serial.println(MyMotor_1.read_velocity());
  Serial.println("2 Velocity (RPM): ");
  Serial.println(MyMotor_2.read_velocity());
  Serial.println();

>>>>>>> develop
  // Serial.println("Velocity (RPM - raw): ");
  // Serial.println(MyMotor.read_raw_velocity());

  // Serial.println("Torque (\% of rated)");
  // Serial.println(MyMotor.read_torque());

<<<<<<< HEAD
  Serial.println("Current (A): ");
  Serial.println(MyMotor.read_current());
=======
  // Serial.println("Current (A): ");
  // Serial.println(MyMotor.read_current());
>>>>>>> develop

  // Serial.println("Voltage (V): ");
  // Serial.println(MyMotor.read_voltage());

  // Serial.println("Temperature (deg C): ");
  // Serial.println(MyMotor.read_temperature());

  // Serial.println("Over Load Ratio (%): ");
  // Serial.println(MyMotor.read_over_load_ratio());

  // Serial.println("Regen Load Ration (%): ");
  // Serial.println(MyMotor.read_regen_load_ratio());
<<<<<<< HEAD
  delay(100);
=======
  delay(500);
>>>>>>> develop
}
