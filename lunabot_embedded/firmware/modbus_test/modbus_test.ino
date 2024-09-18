#include "StepperLib.h"

#define MyMotor 1

void setup(void) {
  Serial.begin(115200);
  Serial.println("Serial connected");

  setup_motors(9600);

  // SAMPLE 1
  move_to_abs_pos(MyMotor, 200000);
  write_estop(MyMotor);

  // SAMPLE 2
  move_to_rel_pos(MyMotor, 10000);
  write_estop(MyMotor);

  // SAMPLE 3
  move_at_speed(MyMotor, 600);
  write_estop(MyMotor);

  // SAMPLE 4 ish
  move_to_abs_pos(MyMotor, -200000);
  write_estop(MyMotor);
}

void loop(void) {
}
