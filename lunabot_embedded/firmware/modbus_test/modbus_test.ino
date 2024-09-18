#include "StepperLib.h"

#define MyMotor 1

void setup(void) {
  Serial.begin(115200);
  Serial.println("Serial connected");

  setup_motors(9600); // TODO - baud rate

  Serial.println("Sample 1:");
  move_to_abs_pos(MyMotor, 200000);
  write_estop(MyMotor);

  Serial.println("Sample 2:");
  move_to_rel_pos(MyMotor, 10000);
  write_estop(MyMotor);

  Serial.println("Sample 3:");
  move_at_speed(MyMotor, 600);
  write_estop(MyMotor);

  Serial.println("Sample 4 (ish):");
  move_to_abs_pos(MyMotor, -200000); // feeding a negative number is fine
  write_estop(MyMotor);

  Serial.println("Sample 1 (no defaults):");
  move_to_abs_pos(MyMotor, 200000, 600, 50, 50); // manually setting the default params
  write_estop(MyMotor);
}

void loop(void) {
}
