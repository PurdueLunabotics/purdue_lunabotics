#include "StepperLib.h"

#define MyMotor 1

void setup(void) {
  Serial.begin(115200);
  Serial.println("Serial connected");

  write_estop(MyMotor);
}

void loop(void) {
}
